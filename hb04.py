#!/usr/bin/python

"""

hb04.py - a python driver for the HB04 CNC pendant device.

Much help from the linuxCNC driver for same.

"""
__author__ = 'Irritant in Residence'
__version__ = '$Revision: 1.0 $'[11:-2]
__copyright__ = 'Copyright (c) 2018 Irritant in Residence'
__license__ = 'Python'


import sys
import math
import threading    # https://docs.python.org/3/library/threading.html
import time
import errno
import atexit
import traceback
import abc

debug = False
debugDisplay = False

try:
    import usb.core
except:
    print("### Missing pyusb.\nTry: sudo pip install pyusb")
    sys.exit(1)


class Pendant(object, metaclass=abc.ABCMeta):
    """abstract what a pendant provides/does"""
    @abc.abstractmethod
    def set_event_handler(self, handler):
        pass
    @abc.abstractmethod
    def get_events_types(self):
        pass
    @abc.abstractmethod
    def axis_value(self, axis_name, wc, mc):
        pass
    @abc.abstractmethod
    def display_attributes(self):
        pass
    @abc.abstractmethod
    def set_display_attribute(self, attribute):
        pass
    @abc.abstractmethod
    def isRunning(self):
        pass
    @abc.abstractmethod
    def disconnect(self):
        pass



class Axis:
    demo = False
    """ HB04 Harware axis switch selects several 'axis'-es; abstract them into one called Axis """
    def __init__(self, name, integerAxis=False):
        self.reset()
        self.integerAxis = integerAxis
        self.name = name

    def __repr__(self):
        """ how we represent this object when printed """
        return f"{self.name}(wc{self.work_coordinate}/mc{self.machine_coordinate} {'int' if self.integerAxis else 'float'})"

    def reset(self):
        self.work_coordinate = 0.0      # this is just a number centered on 0.0;
        self.machine_coordinate = 0.0   # we will assume it is in inches, and convert to metric when needed.
                                        # the actual numbers will come from the motion controller
        self.integerAxis = False        # display as singned 16bit integer, or as two fixed digits floating point
       
    @property
    def wc_display(self):
        """ proper data for updating the work coordinates """
        return self.display(self.work_coordinate)
    @property
    def mc_display(self):
        """ proper data for updating the machine coordinates """
        return self.display(self.machine_coordinate)

    def display(self, v):
        """ format the display value accordinly """
        if self.integerAxis:
            return self.encode_s16(v)
        return self.encode_float(v)

    @property
    def actual(self):
        """" alias for feed/spindle values """
        return self.work_coordinate 

    @property
    def setrate(self):
        """ alias for feed/spindle values """
        return self.machine_coordinate

    @property
    def actual_display(self):
        """" alias for feed/spindle values """
        return self.display(self.actual)

    @property
    def setrate_display(self):
        """ alias for feed/spindle values """
        return self.display(self.setrate)

    def encode_float(self, v):
        """ for the HB04, each numeric display is set using 4 bytes :  """
        int_v = round(math.fabs(v)*10000.0)         # floating point absolute value, rounded off to .00001
        int_part = int(int_v / 10000) & 0xFFFF      # split into 16 bits of integer part (left of decimal)
        fract_part = int(int_v % 10000) & 0xFFFF    # split into 16 bits of decimal part (right of decimal)
        if v < 0:
            fract_part = fract_part | 0x8000        # re-insert 16 bit negative sign on the fract part for negative
        result = list(int_part.to_bytes(2,'little'))# express integer part as 2 byte array of 'little' endianess: lsb, msb
        result.extend( list(fract_part.to_bytes(2,'little')) ) # add same with the fractional part to make 4 byte array
        return result
        
    def encode_s16(self, v):
        """ for the HB04, each feed/spindle display is set using signed 2 bytes (aka, a short-int in little endian) """
        v = int(self.limit_num(v))
        return v.to_bytes(2, "little", signed=True)

    def limit_num(self, x):
        """ a value to this axis's max_units """
        if self.integerAxis:
            max_units = 0xFFFF
            x = int(x)
            if x < 0:
                x = 0
        else:
            max_units = 9999.9999 # max digist on HB04 display when HB04 is in inch mode
        if x > max_units:
            return max_units
        if x < -max_units:
            return -max_units
        return x

def sign_extend(value):
    """ utility funtion to ensure value is negative if 8th bit is set (signed byte) """
    if value > 127:
        return (256-value) * (-1)
    else:
        return value


class hb04(Pendant):
    """
    The HB04 is a Chineese CNC (Computer Numeric Control) Pendant device either conected via USB or WiFi. 
    A pendant is like your TV's remote, but for a computer controlled machining device like a lathe or mill.

    This class wraps a single device in a python wrapper.  The current implementation does not include communication
    to any other devices, and therefore, just a demo of the USB device and it's display.

    More work is required to attach it to motion control, and get real numbers
    """
    HB04VendorID = 0x10CE
    HB04DeviceID = 0xEB70

    def __init__(self):
        atexit.register(self.disconnect)
        self.updateLock = threading.Lock()
        self.reset()        # initial values to reset
        # connect to our USB device
        threading.Thread(target=self.connect_hb04, name='HB04 connect').start()

    def reset(self):
        """ reset the state of the HB04 """
        self.button_names = {   # hex codes returned by button name
            '': 0X0,
            'reset'  : 0x17, 'stop'  : 0x16,
            'go-zero': 0x01, 'start' : 0x02, 'rewind': 0x03, 'probe-z': 0x04,
            'spindle': 0x0C, 'half'  : 0x06, 'zero'  : 0x07, 'safe-z' : 0x08,
            'go-home': 0x09, 'macro1': 0x0A, 'macro2': 0x0B, 'macro3' : 0x05,
            'step'   : 0x0D, 'mode'  : 0x0E, 'macro6': 0x0F, 'macro7' : 0x10
        }
        self.multiplier_values = { # hex codes used to select displayed text next to the multiplier (*1x)
            ''    :0x0,
            '1'   :0x1,   '5':0x2, 
            '10'  :0x3,  '20':0x4,  '30':0x5, '40':0x6, '50':0x7,
            '100' :0x8, '500':0x9,
            '1000':0xA,
            'P6'  :0xB # have no idea what P6 means, but that's what the display will show if you use B
        }
        self.icon_values = { # hex codes used to select the display's icons
            'no-icon':0x0,
            'go-zero':0x10, # matches icon on go-zero button
            'no-touchoff':0x20, # funny work/tool icon with a circle and line thru it
            'go-home':0x50, # matches icon on the go-home button
            'j-squiggily':0x60 # large "J" with saw-tooth icon 
            }
        self.units_values = { # hex codes to select the mm and inch display icons
            'mm':0,
            "inch":0x80 # selecting this also adds an additional digit to the fractional display for xyza
            }
        self.axis_select_switch = { # hex codes returned from the axis selector switch
            'off'    : 0x00, # the switch is break-before-make, so "off" selection will come between axis switches.
            'x'      : 0x11, # if you turn the switch fast enough, though, the interviening "off" is missing.
            'y'      : 0x12,
            'z'      : 0x13,
            'a'      : 0x18,
            'spindle': 0x14,
            'feed'   : 0x15
        }

        # build inverse tables
        self.code2Button = { v:k for k,v in self.button_names.items() }
        self.code2Axis = { v:k for k,v in self.axis_select_switch.items() }
        self.code2Range = { v:k for k,v in self.multiplier_values.items() }

        # build all the axis-es (eg. self.x, self.feed, etc.)
        self.axis_list = {}     
        for name in 'x y z a'.split(' '):
            axis = Axis(name)
            setattr(self, name, axis)
            self.axis_list[name]=axis
        for name in 'feed feed-actual spindle spindle-actual'.split(' '):
            axis = Axis(name,integerAxis=True)
            setattr(self, name, axis)
            self.axis_list[name]=axis

        ## hardware power-up states (display attributes)
        self.icon = 0            # no-icon
        self.units = 0           # mm
        self.multiplier_code = 1 # 0

        # off!! -- hurm... need to figure switch position before we get 1st update... TODO!
        self.axis_switch = 'off'    

        # where button events go
        self.machine_event_handler = None
        
        # if we need to reattach kernel usb handler
        self.reattach = False
        
        # we're not in the process of reconnecting
        self.reconnecting = False
        self.running = False



    def __repr__(self):
        stat = "HB04:"
        for name,axis in self.axis_list.items():
            stat += f"{axis} "
        stat += f"icon:{self.icon} "
        stat += f"units:{self.units} "
        stat += f"x1:{self.multiplier_code} "
        stat += f"a-sw:{self.axis_switch}"
        return stat
       
    def connect_hb04(self):
        """return when a good HB04 is found via USB"""
        once = False
        self.disconnecting = False
        while 1:
            if self.disconnecting == True:
                return
            self.dev = usb.core.find(idVendor=self.HB04VendorID, idProduct=self.HB04DeviceID)    # find our device
            if self.dev:
                break    # found
            if not once:
                print("HB04 not found. Please attach to continue...")
                once = True
            time.sleep(.1)

        # grab from kernel so we can use it if necessary.
        if self.dev.is_kernel_driver_active(0):
            self.reattach = True   # if we were being a good citizen, we'd reattach when we were done.. but we're not
            self.dev.detach_kernel_driver(0)

        try:# if this fails, then we couldn't find a good device USB endpoint to listen to
            # this is the inbound button data USB endpoint ... 
            self.endpoint_in = self.dev[0][(0,0)][0] # go figure that syntax out!
        except:
            self.fatal("Found, but failed to connect to HB04")

        print("Found HB04")
        self.startListening() # start listening
        self.forceDisplayUpdate()

    def fatal(self, msg):
        print(f"### FATAL: {msg}")
        traceback.print_exc(file=sys.stdout)
        self.disconnect()
        sys.exit(1)

    def disconnect(self):
        self.running = False
        self.disconnecting = True
        if self.reattach:
            try:
                if not self.dev.is_kernel_driver_active(0):
                    self.dev.attach_kernel_driver(0)
            except:
                pass

    def isRunning(self):
        return self.running or self.reconnecting
        
    def startListening(self):
        """
        Starts a thread to listen for USB messages from HB04, and other thread to update the HB04 display.
        We don't want to wait for them since events from the USB device are asyncronous.
        """
        self.running = True
        threading.Thread(target=self.receiveUSBEvents, name='HB04 listener').start()
        threading.Thread(target=self.displayPeriodically, name='HB04 display updater').start()

    def reconnect(self):
        self.reconnecting = True
        self.running = False
        time.sleep(.5) # let threads die
        self.reconnecting = False
        self.connect_hb04()

    def receiveUSBEvents(self):
        """ ON THREAD:: continually wait for USB messages and then process them """
        if debug: print("HB04 listener running")
        while self.running:
            # catch any USB errors
            try:
                data = self.dev.read(self.endpoint_in.bEndpointAddress, 64, 1000)
            except Exception as e:
                if isinstance(e, usb.core.USBError):
                    if e.errno == errno.ETIMEDOUT: # timeout
                        continue
                    if e.errno == errno.ENODEV:   # no device
                        print("### USB Device disconnect")
                        time.sleep(2)
                        self.connect_hb04()
                        self.forceDisplayUpdate()
                        continue
                    if e.errno == errno.EBUSY:
                        print("### Device in use by another program")
                        time.sleep(1)
                    if debug: print(f"## HB04 USB error - reconnecting")
                    threading.Thread(target=self.reconnect).start()
                    break
                continue

            try:
                # catch any of my errors
                self.newUSBEvent(data) # process new data
            except:
                self.fatal("### Programmer error!!###")
        if debug: print("### STOP hb04 listening")


    def newUSBEvent(self, data):
        """
        ON THREAD:: process inbound USB data
        data[0[ is always 0x04
        data[1] is button pressed #1
        data[2] is button pressed #2
        data[3] contains the axis switch position
        data[4] axis increment
        data[5] appears to be as copy of data[1]
        """
        assert len(data) == 6
        
        self.axis_switch = self.code2Axis[data[3]]

        try:
            if self.machine_event_handler:
                self.machine_event_handler({
                    'button':self.code2Button[data[1]],
                    'button2':self.code2Button[data[2]],
                    'axis':self.axis_switch,
                    'increment':sign_extend(data[4])
                    })
        except:
            pass
  
    def forceDisplayUpdate(self):
        """ update display """
        try:
            self.updateDisplay()
        except Exception as e:
            self.fatal(f"updateDisplay failing with {e}")

    def displayPeriodically(self, period=1/3):
        """ON THREAD:: update the display every 1/3 second """
        lastCounter = 0
        while self.running:
            self.forceDisplayUpdate()
            time.sleep(period) # Periodically
        if debug: print("### STOP hb04 updating display")

        
    def updateDisplay(self):
        """
        Take state data and drive HB04 display update
        Builds the command bytes for an update
        """
        # magic numbers: I don't know what these do, perhaps vendor info
        data = [ 0xFE, 0xFD, 0x0C ]

        if debugDisplay: print(f"updateDisplay - {self}")

        """
        The rest of the data is:
        6 sets of 4 byte floats (a/x,y,z work, a/x,y,z machine)
        4 sets of 2 byte shorts (feed,spindle actual, then set)
        1 byte of control for other things like icons etc.
        1 byte for in/mm indication
        """
        
        if self.axis_switch == 'a':
            data.extend( self.a.wc_display )
        else:
            data.extend( self.x.wc_display )
        data.extend( self.y.wc_display )
        data.extend( self.z.wc_display )
        if self.axis_switch == 'a':
            data.extend( self.a.mc_display )
        else:
            data.extend( self.x.mc_display )
        data.extend( self.y.mc_display )
        data.extend( self.z.mc_display )
    
        data.extend( self.feed.actual_display )
        data.extend( self.spindle.actual_display )
        data.extend( self.feed.setrate_display )
        data.extend( self.spindle.setrate_display )

        data.append( self.icon | self.multiplier_code )
        data.append( self.units )    # should be byte number 37

        ## what are bytes 37-42 used for??

        with self.updateLock: # avoid reentrancy
            self.sendData(data)

    def sendData(self, data):
        """Send the raw data to the USB device's display"""
        if debugDisplay:
            print("DisplayUpdate data: {}".format(data))
        while len(data) < 42: # 6 packets of 7 bytes
            data.append(0)  # pad out data to make the loop easy

        # Break 7 bytes of data into packets of 8 bytes with a 0x06 lead-in character
        packets = []
        data_idx = 0
        # the raw data is sent in 6 eight byte packets
        for packet in range(6):
            for i in range(8):
                if i == 0:
                    pd = list([0x6])   # each packet starts with 0x06 byte
                    packets.append(pd)
                else:
                    pd.append( data[data_idx] )  # followed by the next 7 bytes of raw data
                    data_idx += 1

        # send the packets over USB to the device
        for p in packets:
            _data = bytes(p)    # send bytes, not characters
            if debug and 0:
                print("Packet: {}".format(_data))
            try:
                # the first 4 arguments to ctrl_transfer are:
                # bmRequestType : 0x21 - means from host to device, so nothing special here
                # bmRequest : 0x09 - device specific value
                # wValue : 0x306 - device specific value
                # wIndex : 0x00 - device specific value
                result = self.dev.ctrl_transfer(0x21, 0x09, 0x0306, 0x00, _data, 1000)
                assert result == len(_data)
            except Exception as e:
                # just ignore device update failures, consider them transient
                if debug: print(f"Exception while sending packet {e}")


##### Implement the Pendant Interface #####
    def set_event_handler(self, handler):
        self.machine_event_handler = handler
    
    def get_events_types(self):
        return ['button', 'button2', 'axis', 'increment']

    def axis_value(self, axis_name, wc=None, mc=None):
        if 0: print(f"Setting axis-{axis_name} to wc:{wc} mc:{mc}")
        try:
            if axis_name in self.axis_list:
                axis = self.axis_list[axis_name]
                if wc is not None:
                    axis.work_coordinate = wc
                if mc is not None:
                    axis.machine_coordinate = mc
            else:
                self.fatal(f"### Can't find axis {axis_name}")
        except:
            pass
        self.forceDisplayUpdate()

    def display_attributes(self):
        attrs = []
        attrs.extend( self.units_values.keys() )
        attrs.extend( self.icon_values.keys() )
        attrs.extend( self.multiplier_values.keys() )
        return attrs
    
    def set_display_attribute(self, attribute):
        if debug: print(f"set_display_attribute:{attribute}")
        if not attribute in self.display_attributes():
            print(f"### HB04 doesn't have the display attribute {attribute} : {value}")
            return
        if attribute in self.units_values:
            self.units = self.units_values[attribute]
        elif attribute in self.icon_values:
            self.icon = self.icon_values[attribute]
        elif attribute in self.multiplier_values:
            self.multiplier_code = self.multiplier_values[attribute]
        else:
            msg = f"### Can't happen -- missing {attribute}"
            raise RuntimeError(msg)
        self.forceDisplayUpdate()


#### DEMO machine
def check_power(N, k):
    """checks is N is a power of k"""
    if N == k:
        return True
    try:
        return N == k**int(round(math.log(N, k)))
    except Exception:
        return False


class hb04_demo:
    def __init__(self):
        self.lastButtons = {'button':None, 'button2':None}
        self.scale = 1 # 1x
        self.units = 'mm'
        self.axis_name = 'off'
        self.buttons = {
            'step': self.step,
            'zero': self.zero,
            'mode': self.mode,
            'go-zero': self.go_zero,
            'go-home': self.go_home,
            'probe-z': self.probez,
            'safe-z': self.safez,
        }
        self.lastButtons = {'button':None, 'button2':None}
        self.p = hb04()
        self.p.set_event_handler(self.pendantEvents)
        time.sleep(1) # let it start up
        if self.p.isRunning():
            print("HB04 Demo is running.\nAll activity is now on the HB04 device.\n Nothing to see here.\n^C to exit.")
        else:
            print("Sorry, there seems to be something wrong with the demo...")

    def pendantEvents(self, event):
        try:
            self._pendantEvents(event)
        except:
            self.p.fatal("### WHOOPS! I did a boo boo")
            
    def _pendantEvents(self, event):
        if debug: print(f"pendantEvent:{event}")
        try:
            if 0: print(self.lastButtons)
            for k in 'axis button button2 increment'.split(' '):
                if not k in event:
                    continue
                v = event[k]
                if k == 'button' or k == 'button2':
                    if not v and self.lastButtons[k]:
                        if debug: print(f"PENDANT BUTTON UP:: {k}:{self.lastButtons[k]}", flush=True)
                        self.pendantButton(self.lastButtons[k], pressed=False)
                        self.lastButtons[k] = None
                    elif v and v != self.lastButtons[k]:
                        if debug: print(f"PENDANT BUTTON DOWN:: {k}:{v}", flush=True)
                        self.lastButtons[k] = v
                        self.pendantButton(v)

                if k == 'axis' and v and v != 'off':
                    if 0: print(f"PENDANT AXIS:: {v}")
                    self.axis_name = v

                if k == 'increment' and v:
                    if debug: print(f"PENDANT INCREMENT:: {self.axis_name} by {v}")
                    self.rawIncrement(self.axis_name, v)
        except Exception as e:
            self.g2.fatal(f"pendantEvent fails {event}")

    def pendantButton(self, button, pressed=True):
        if button in self.buttons:
            if pressed:
                self.p.set_display_attribute('no-icon')
            self.buttons[button](pressed)
    
    def zero(self, pressed):
        if pressed:
            getattr(self.p, self.axis_name).work_coordinate = 0

    def mode(self, pressed):
        """ switch mm/inch """
        if pressed:
            self.units = 'inch' if self.units == 'mm' else 'mm'
            self.p.set_display_attribute(self.units)
    def go_zero(self, pressed):
        if pressed: self.p.set_display_attribute('go-zero')
    def go_home(self, pressed):
        if pressed: self.p.set_display_attribute('go-home')
    def probez(self, pressed):
        if pressed: self.p.set_display_attribute('no-touchoff')
    def safez(self, pressed):
        if pressed: self.p.set_display_attribute('j-squiggily')
            
    def step(self, pressed):
        """ change the scale factor for the knob """
        if not pressed:
            return
        scales = []
        for item in self.p.display_attributes():
            try:
                i = int(item)
                if i:
                    scales.append(i)
            except:
                pass # wasn't an integer
        for scale in sorted(scales):
            # we're only interested in going up, and by 1, 10, 100, or 1000
            if scale > self.scale and check_power(scale, 10):
                break
        else:
            scale = 1
        
        self.scale = scale
        if 0: print(f"Setting scale to {scale}")
        self.p.set_display_attribute(str(scale))
        
    def rawIncrement(self, axis_name, v):
        if axis_name in "x y z a feed spindle".split():
            if axis_name == 'a': # in degrees/min/sec/8
                v /= 1000 # s/b 60 * 60 * 8
            elif len(axis_name)==1:
                v /= 10000 #prescale
                v *= self.scale*10
            try:
                getattr(self.p, axis_name).work_coordinate += v
                getattr(self.p, axis_name).machine_coordinate += v
            except:
                print(f"### increment unknown axis {axis_name}")
        else:
            print(f"### increment unknown axis {axis_name}")
            raise # fix feed axis


    
if __name__ == "__main__":
    hb04_demo()
