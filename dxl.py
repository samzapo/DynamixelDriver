import pypot.dynamixel
ports = pypot.dynamixel.get_available_ports()
baud = [57142, 1000000, ]

for p in ports:
    for b in baud:
        with pypot.dynamixel.DxlIO(p, b) as dxl:
            print 'opened port {} with baudrate {} and dectected motor ids:{}'.format(p, b, dxl.scan(range(253)))
