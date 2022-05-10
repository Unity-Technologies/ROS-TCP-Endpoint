import sys
import roslib.message

if len(sys.argv) != 2:
    print("Usage: python message_type_info.py <messagename>\nFor example, python message_type_info.py std_msgs/String")
else:
    message_class = roslib.message.get_message_class(sys.argv[1])
    if message_class == None:
        sys.stderr.write("ERROR: unknown message type: "+sys.argv[1] + "\n")
        sys.exit(1)
    else:
        print(message_class._md5sum)
        print(message_class._full_text, end='')

