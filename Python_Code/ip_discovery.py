from netifaces import interfaces, ifaddresses, AF_INET


def ip_find(net_detail):
    ip_values = []
    for details in net_detail:
        ip = details.get('addr', None)
        if (not '127' in ip) and (ip is not None):
            ip_values.append(ip)
    return ip_values

def find_connected_networks(interfaces):
    connected_ips= []
    for ifaceName in interfaces:
        try:
            val = ifaddresses(ifaceName)[AF_INET]
            connected_ips.extend(ip_find(val))
        except KeyError:
            pass  
    return connected_ips

def ip_selection(interfaces):
    ip_table = find_connected_networks(interfaces)
    print("Connected IP's: ")
    for i in range(len(ip_table)):
        print(f"{i} - {ip_table[i]}")
    while 1:
        
        ip_index = input("Which IP do you with to use?\n")
        
        if ip_index == 'E' or ip_index == 'e':
            ip = None
            break
        try:
            ip = ip_table[int(ip_index)]
            break
        except:
            print("Not a valid index value try again or type E/e and exit the loop.")
    return ip
