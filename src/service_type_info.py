import sys
import roslib.message

if len(sys.argv) != 2:
    print("Usage: python service_type_info.py <servicename>\nFor example, python service_type_info.py std_srvs/SetBool")
else:
    service_class = roslib.message.get_service_class(sys.argv[1])
    if service_class == None:
        sys.stderr.write("ERROR: unknown service type: "+sys.argv[1] + "\n")
        sys.exit(1)
    else:
        print(service_class._md5sum)
