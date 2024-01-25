#
# Ethercat management for firmware communication
#

# imports
import logging, threading, os, sys
# add klippy dependency
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'klippy'))
import msgproto, chelper, util

class error(Exception):
    pass

class EthercatReader:
    def __init__(self, reactor, warn_prefix=""):
        '''
        Ethercat reader.
        '''
        self.reactor = reactor #main reactor
        self.warn_prefix = warn_prefix
        # message parser (ethercat specific)
        self.msgparser = msgproto.MessageParser(warn_prefix=warn_prefix)
        # C interface
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.ethcatqueue = None #ethcatqueue (main structure for automatic EtherCAT communication)
        self.stats_buf = self.ffi_main.new('char[4096]') #status buffer for debug purpose
        # threading
        self.lock = threading.Lock() #mutex to access response queue of low level thread
        self.background_thread = None #high level ethercat thread (process drive responses)
        # message handlers
        self.handlers = {}
        self.register_response(self._handle_unknown_init, '#unknown')
        self.register_response(self.handle_output, '#output')
        # notification tracking
        self.last_notify_id = 0
        self.pending_notifications = {}
        
    def _bg_thread(self):
        '''
        Ethercat high level thread running in background and handling pre-processed
        responses coming from the low level ethercat thread. The EtherCAT stack
        handles most of the communication logic, the responses coming up to here
        are only special cases that require further processing or notification.
        '''
        # response message
        response = self.ffi_main.new('struct pull_queue_message *')
        while 1:
            # get and remove first message from receive queue
            self.ffi_lib.ethcatqueue_pull(self.ethcatqueue, response)
            count = response.len
            if count < 0:
                # stop (error)
                break
            if response.notify_id:
                logging.info("LLLLLLL = %s", response)
                ''' 
                Response require notification, handle it asynchronously.
                '''
                # create message parameters
                params = {'#sent_time': response.sent_time, '#receive_time': response.receive_time}
                completion = self.pending_notifications.pop(response.notify_id)
                self.reactor.async_complete(completion, params)
                continue
            # get message parameters from encoded message
            params = self.msgparser.parse(response.msg[0:count])
            params['#sent_time'] = response.sent_time
            params['#receive_time'] = response.receive_time
            # message handler key
            hdl = (params['#name'], params['oid']) #high level response handler
            try:
                with self.lock:
                    # lock handler (avoid registrations in the meantime)
                    hdl = self.handlers.get(hdl, self.handle_default)
                    # run message handler
                    hdl(params)
            except:
                logging.exception("%sException in ethcat callback", self.warn_prefix)
                
    def _error(self, msg, *params):
        '''
        Raise error.
        '''
        raise error(self.warn_prefix + (msg % params))
                
    def _start_session(self):
        '''
        Start new session, there is no direct contact with the mcu during the setup.
        '''
        # allocate ehtercatqueue and start low level thread
        self.ethcatqueue = self.ffi_main.gc(self.ffi_lib.ethcatqueue_alloc(), self.ffi_lib.ethcatqueue_free)
        # create and start high level thread
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start() #start high level background thread
        # create ethercat private message parser
        msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
        self.msgparser = msgparser
        # obtain and load drive specific data dictionary
        identify_data = None
        try:
            with open('./commands/drive_commands.json', 'r') as command_file:
                identify_data = command_file.read()
        except:
            logging.info("%sError in reading ethercat command file.", self.warn_prefix)
            self.disconnect()
            return False
        # process ethercat commands and responses on host side
        if identify_data is not None:
            logging.info("%sLoading EtherCAT data dictionary ...", self.warn_prefix)
            msgparser.process_identify(identify_data, decompress=False)
        # unknown response handler (default)
        self.register_response(self.handle_unknown, '#unknown')
        return True
    
    def connect_ethercat(self):
        '''
        Connect with EtherCAT slaves. A fixed slave configuration is given,
        therefore there is no need to pause and read/write additional
        object dictionary entries.
        '''
        logging.info("%sStarting ethercat connect...", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while 1:
            if self.reactor.monotonic() > start_time + 90.:
                # stop (connection timeout)
                self._error("Unable to connect to EtherCAT.")
            # start new session (high and low level threads)
            ret = self._start_session()
            if ret:
                logging.info("%sEthercat connected.", self.warn_prefix)
                break
        
    def set_clock_est(self, freq, conv_time, conv_clock, last_clock):
        '''
        Set drive clock estimate. This clock is continuously synchronized
        with the mcu one through the serial module (not directly).
        '''
        if self.ethcatqueue is not None:
            self.ffi_lib.ethcatqueue_set_clock_est(self.ethcatqueue, freq, conv_time, conv_clock, last_clock)
        
    def disconnect(self):
        '''
        Disconnect EtherCAT communication with drive.
        '''
        if self.ethcatqueue is not None:
            self.ffi_lib.ethcatqueue_exit(self.ethcatqueue)
            if self.background_thread is not None:
                self.background_thread.join()
            self.background_thread = self.ethcatqueue = None
        for pn in self.pending_notifications.values():
            pn.complete(None)
        self.pending_notifications.clear()
        
    def stats(self, eventtime):
        '''
        Extract old messages stored in the debug queue.
        '''
        if self.ethcatqueue is None:
            return ""
        self.ffi_lib.ethcatqueue_get_stats(self.ethcatqueue, self.stats_buf, len(self.stats_buf))
        return str(self.ffi_main.string(self.stats_buf).decode())
    
    def get_reactor(self):
        '''
        Get reactor.
        '''
        return self.reactor
    
    def get_msgparser(self):
        '''
        Get ethercat message parser.
        '''
        return self.msgparser
    
    def get_ethcatqueue(self):
        '''
        Get ethercat queue.
        '''
        return self.ethcatqueue
    
    def get_default_command_queue(self):
        '''
        Get ethercat default tx command queue (request queue), it is
        already part of ethercatqueue but keep it for compatibility.
        '''
        return None
    
    def register_response(self, callback, name, oid=None):
        '''
        Register response callbacks. NOTE: register a response for each low
        level drive response, it has to be in the same format of the serial
        one since the same reactor and message parser structure are used.
        The oid is unique for each message parser but serial and ethercat
        can use the same oid for different handlers since they are separate.
        '''
        with self.lock:
            if callback is None:
                del self.handlers[name, oid]
            else:
                self.handlers[name, oid] = callback
                
    def raw_send(self, cmd, minclock, reqclock, cmd_queue=None):
        '''
        Send raw command.
        '''
        logging.info("DIOBRACA CMD = %s", cmd)
        # add command message to request queue
        self.ffi_lib.ethcatqueue_send_command(self.ethcatqueue, cmd, len(cmd),
                                              minclock, reqclock, 0)
        
    def raw_send_wait_ack(self, cmd, minclock, reqclock, cmd_queue=None):
        '''
        Send raw command and wait for confirmation.
        '''
        logging.info("DIOBRACA QUERY CMD = %s", cmd)
        self.last_notify_id += 1
        nid = self.last_notify_id
        # get available greenlet from reactor
        completion = self.reactor.completion()
        # add request to waiting list
        self.pending_notifications[nid] = completion
        # add command message to request queue
        self.ffi_lib.ethcatqueue_send_command(self.ethcatqueue, cmd, len(cmd),
                                              minclock, reqclock, nid)
        # wait for completion
        params = completion.wait()
        logging.info("DIOBRACA RES = %s", params)
        if params is None:
            self._error("Ethercat connection closed")
        return params
    
    def send(self, msg, minclock=0, reqclock=0):
        '''
        Send a command request.
        '''
        cmd = self.msgparser.create_command(msg)
        self.raw_send(cmd, minclock, reqclock, None)
        
    def send_with_response(self, msg, response):
        '''
        Send command and wait for response.
        '''
        cmd = self.msgparser.create_command(msg)
        src = EthercatRetryCommand(self, response)
        return src.get_response([cmd], None)
    
    def alloc_command_queue(self):
        '''
        Allocate command queue.
        '''
        return self.ffi_main.gc(self.ffi_lib.ethcatqueue_alloc_commandqueue(),
                                self.ffi_lib.ethcatqueue_free_commandqueue)
    
    def _handle_unknown_init(self, params):
        logging.debug("%sUnknown message %d (len %d) while identifying", self.warn_prefix, params['#msgid'], len(params['#msg']))
        
    def handle_unknown(self, params):
        logging.warn("%sUnknown message type %d: %s", self.warn_prefix, params['#msgid'], repr(params['#msg']))
        
    def handle_output(self, params):
        logging.info("%s%s: %s", self.warn_prefix, params['#name'], params['#msg'])
        
    def handle_default(self, params):
        logging.warn("%sgot %s", self.warn_prefix, params)


class EthercatRetryCommand:
    '''
    Class to send a query command and return the received response.
    '''
    def __init__(self, ethcat:EthercatReader, name, oid=None):
        self.ethcat = ethcat
        self.name = name
        self.oid = oid
        self.last_params = None
        self.ethcat.register_response(self.handle_callback, name, oid)
        
    def handle_callback(self, params):
        '''
        Store response from reactor handler in caller object so that
        the associated drive can read it later.
        '''
        self.last_params = params
        
    def get_response(self, cmds, cmd_queue=None, minclock=0, reqclock=0):
        '''
        Send buffered commands and wait for response.
        '''
        retries = 5 #max number of retries before error
        retry_delay = .010 #delay between retries (in seconds)
        while 1:
            for cmd in cmds[:-1]:
                # send simple (no ack) commands
                self.ethcat.raw_send(cmd, minclock, reqclock, cmd_queue)
            # wait for response to the last command
            self.ethcat.raw_send_wait_ack(cmds[-1], minclock, reqclock, cmd_queue)
            params = self.last_params
            if params is not None:
                # response received (remove handler from reactor)
                self.ethcat.register_response(None, self.name, self.oid)
                return params
            if retries <= 0:
                # timeout (remove handler from reactor)
                self.ethcat.register_response(None, self.name, self.oid)
                raise error("Unable to obtain '%s' response" % (self.name,))
            reactor = self.ethcat.reactor
            reactor.pause(reactor.monotonic() + retry_delay)
            retries -= 1
            retry_delay *= 2. #double retry delay