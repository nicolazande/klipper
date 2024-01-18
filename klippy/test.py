import msgproto

# obtain and load drive specific data 
with open('./commands/drive_commands.json', 'r') as command_file:
    identify_data = command_file.read()

msgparser = msgproto.MessageParser(warn_prefix="mcu")

msgparser.process_identify(identify_data, decompress=False)

for key, val in msgparser.messages_by_id.items():
    print("msgid = %s" % val.msgid)
    print("name = %s" % val.name)
    print("param names = ")
    for i in val.param_names:
        print(i)
    print("param types = ")
    for i in val.param_types:
        print(i)
