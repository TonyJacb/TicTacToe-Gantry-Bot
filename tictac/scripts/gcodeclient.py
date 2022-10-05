import serial
import time

#Inspired from 
#https://kevinponce.com/blog/python/send-gcode-through-serial-to-a-3d-printer-using-python/

class Client:
    def __init__(self, port, baud) -> None:
        '''
        Initialises the serial port and wakes up GRBL with desired settings.
        :param port: specify the port to which your printer is connected.
                    If it is an Arduino CNC shield, check the port from Arduino IDE
        :param baud: specify the baudrate at which GRBL is communicating. 
        '''
        self.ser = serial.Serial(port, baud)
        time.sleep(2)

        #Keeping track of the position in absolute values
        self.value_X = 0.0
        self.value_Y = 0.0
        self.crosses = []

        self.__initialise("G28")
        # self.__initialise("G28 X0 Y0 Z0\r\n")
        # self.__initialise("G28 X0 Y0\r\n")
        # self.__initialise("G28 X0\r\n")
        # self.__initialise("G28 Y0\r\n")
        # self.__initialise("G28 Z0\r\n")

        # Extruder Temp
        # self.__initialise("M104 S190 T0\r\n") #  start heating T0 to 190 degrees Celsius
        # self.__initialise("G28\r\n") # Home
        # self.__initialise("M109 S190 T0\r\n") # wait for T0 to reach 190 degrees before continuing with any other self.commands

        # Bed Temp
        # self.__initialise("M140 S55\r\n") # heat bed to 50 degrees celsius but do not wait
        # self.__initialise("G28\r\n") # Home
        # self.__initialise("M190 S55\r\n") # wait for bed to heat to 50 degrees celsius and wait

        # Fan
        # self.__initialise("M106 S255\r\n") # fan speed full
        # self.__initialise("M106 S127\r\n") # fan speed about half
        # self.__initialise("M106 S0\r\n") # turn off fan

        # Set Units(does not seem to work on ender 5)
        # self.__initialise("G20\r\n") # inches
        self.__initialise("G21") # millimeters

        # Absolute Mode
        # self.__initialise("G90\r\n")

        # Relative Mode
        self.__initialise("G91\r\n")

        #Feed Rate
        self.__initialise("F260\r\n")

        # Move
        # self.command("G0 X7 Y18\r\n") # rapid motion but does not extrude material
        # self.command("G0 X350 Y350\r\n") # rapid motion but does not extrude material ender 5 plus is 350 x 350
        # self.command("G1 Z0.345 F500\r\n") # change layer
        # self.command("G0 X50 Y50\r\n") # rapid motion but does not extrude material ender 5 plus is 350 x 350

    def command(self, cmd):
        '''
        Interfaces the Gcode commands to GRBL
        :param cmd:  A Gcode String.
        '''
        try:
            cmd = cmd.upper()
            subcmds = cmd.split(" ")
            for subcmd in subcmds:
                if subcmd[0] == "X":
                    self.value_X += float(subcmd[1:])
                elif subcmd[0] == "Y":
                    self.value_Y += float(subcmd[1:])
            # print(f'Value of X: {self.value_X}, y:{self.value_Y}')
            cmd = cmd + "\r\n"
            self.ser.write(str.encode(cmd))
            time.sleep(1)
            while True:
                feedback = self.ser.readline()
                if feedback == b'ok\r\n':
                    # print(feedback)
                    break
        except TypeError:
            print("Gcode commands must be a string")


    def __initialise(self, cmd):
        '''
        Same as that of command but for initialisation. Used in the constructor.
        '''
        cmd = cmd + "\r\n"
        self.ser.write(str.encode(cmd))
        time.sleep(1)
        while True:
            feedback = self.ser.readline()
            if feedback == b'ok\r\n':
                # print(feedback)
                break

    def     flush(self):
        '''
        Use this function to close the serial port.
        '''
        time.sleep(2)
        self.ser.close()
        quit()

    def automate_mode(self, string):
        if string == "0":
            self.goto_origin()
            self.draw_X()
            
        elif string == "1":
            self.goto_one()
            self.draw_X()
            self.goto_origin()
        
        elif string == "2":
            self.goto_second()
            self.draw_X()
            self.goto_origin()

        elif string == "3":
            self.goto_third()
            self.draw_X()
            self.goto_origin()

        elif string == "4":
            self.goto_fourth()
            self.draw_X()
            self.goto_origin()

        elif string == "5":
            self.goto_fifth()
            self.draw_X()
            self.goto_origin()

        elif string == "6":
            self.goto_sixth()
            self.draw_X()
            self.goto_origin()

        elif string == "7":
            self.goto_seventh()
            self.draw_X()
            self.goto_origin()

        elif string == "8":
            self.goto_eight()
            self.draw_X()
            self.goto_origin()


         
    def manual_mode(self):
        '''
        Use this for sending one command at a time.
        '''
        while True:
            string = input("Gcode /Q to quit /R to reset: ")
            string = string.upper()
            print(string)
            if string == "Q":
                self.flush()

            elif string == "R":
                self.value_X = 0
                self.value_Y = 0
                print(f'Value of X: {self.value_X}, Y:{self.value_Y}')
            
            elif string == "H":
                self.goto_origin()
                self.draw_plane()

            elif string == "0":
                self.goto_origin()
            
            elif string == "1":
                self.goto_one()
            
            elif string == "2":
                self.goto_second()

            elif string == "3":
                self.goto_third()

            elif string == "4":
                self.goto_fourth()

            elif string == "5":
                self.goto_fifth()

            elif string == "6":
                self.goto_sixth()

            elif string == "7":
                self.goto_seventh()

            elif string == "8":
                self.goto_eight()
            
            elif string == "X":
                self.draw_X()
                self.check_for_win()

            else:
                self.command(string)
    def goto_origin(self):
        '''
        (X=0,Y=0)
        '''
        x = -self.value_X
        y = -self.value_Y

        x=str(x)
        y=str(y)

        self.command("G01 X"+x+" Y"+y)

    def goto_one(self):
        '''
        (X = 21, Y = 0)
        '''
        x = self.value_X 
        if x > 21 or x < 21:
            x = str(21 - x)
        elif x == 21:
            x = str(0)
        y = str(-self.value_Y)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)
    
    def goto_second(self):
        '''
        (X = 42, Y = 0)
        '''
        x = self.value_X 
        if x > 42 or x < 42:
            x = str(42 - x)
        elif x == 42:
            x = str(0)
        y = str(-self.value_Y)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)

    def goto_third(self):
        '''
        (X = 0 , Y = 10)
        '''
        x = str(-self.value_X)
        y = self.value_Y
        if y > -10 or y < -10:
            y = str(-10 + y)
        elif y == -10:
            y = str(0)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)

    def goto_fourth(self):
        '''
        (X = 21, Y = 10)
        '''
        x = self.value_X 
        if x > 21 or x < 21:
            x = str(21 - x)
        elif x == 21:
            x = str(0)
        y = self.value_Y
        if y > -10 or y < -10:
            y = str(-10 + y)
        elif y == -10:
            y = str(0)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)

    def goto_fifth(self):
        '''
        (X = 42, Y = 10)
        '''
        x = self.value_X 
        if x > 42 or x < 42:
            x = str(42 - x)
        elif x == 42:
            x = str(0)
        y = self.value_Y
        if y > -10 or y < -10:
            y = str(-10 + y)
        elif y == -10:
            y = str(0)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)

    def goto_sixth(self):
        '''
        (X = 0, Y = 20)
        '''
        x = str(-self.value_X)
        y = self.value_Y
        if y > -20 or y < -20:
            y = str(-20 + y)
        elif y == -20:
            y = str(0)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)

    def goto_seventh(self):
        '''
        (X = 21, Y = 20)
        '''
        x = self.value_X 
        if x > 21 or x < 21:
            x = str(21 - x)
        elif x == 21:
            x = str(0)
        y = self.value_Y
        if y > -20 or y < -20:
            y = str(-20 + y)
        elif y == -20:
            y = str(0)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)    

    def goto_eight(self):
        '''
        (X = 42, Y = 20)
        '''    
        x = self.value_X 
        if x > 42 or x < 42:
            x = str(42 - x)
        elif x == 42:
            x = str(0)
        y = self.value_Y
        if y > -20 or y < -20:
            y = str(-20 + y)
        elif y == -20:
            y = str(0)
        print("G01 X"+x+" Y"+y)
        self.command("G01 X"+x+" Y"+y)  

    # def goto_origin(self):
    #     '''
    #     (X=0,Y=0)
    #     '''
    #     x = -self.value_X
    #     y = -self.value_Y

    #     x=str(x)
    #     y=str(y)

    #     self.command("G01 X"+x+" Y"+y)

    # def goto_one(self):
    #     '''
    #     (X = 21, Y = 0)
    #     '''
    #     x = self.value_X 
    #     if x > 21 or x < 21:
    #         x = str(21 - x)
    #     elif x == 21:
    #         x = str(0)
    #     y = str(-self.value_Y)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)
    
    # def goto_second(self):
    #     '''
    #     (X = 42, Y = 0)
    #     '''
    #     x = self.value_X 
    #     if x > 42 or x < 42:
    #         x = str(42 - x)
    #     elif x == 42:
    #         x = str(0)
    #     y = str(-self.value_Y)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)

    # def goto_third(self):
    #     '''
    #     (X = 0 , Y = 10)
    #     '''
    #     x = str(-self.value_X)
    #     y = self.value_Y
    #     if y > 10 or y < 10:
    #         y = str(10 - y)
    #     elif y == 10:
    #         y = str(0)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)

    # def goto_fourth(self):
    #     '''
    #     (X = 21, Y = 10)
    #     '''
    #     x = self.value_X 
    #     if x > 21 or x < 21:
    #         x = str(21 - x)
    #     elif x == 21:
    #         x = str(0)
    #     y = self.value_Y
    #     if y > 10 or y < 10:
    #         y = str(10 - y)
    #     elif y == 10:
    #         y = str(0)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)

    # def goto_fifth(self):
    #     '''
    #     (X = 42, Y = 10)
    #     '''
    #     x = self.value_X 
    #     if x > 42 or x < 42:
    #         x = str(42 - x)
    #     elif x == 42:
    #         x = str(0)
    #     y = self.value_Y
    #     if y > 10 or y < 10:
    #         y = str(10 - y)
    #     elif y == 10:
    #         y = str(0)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)

    # def goto_sixth(self):
    #     '''
    #     (X = 0, Y = 20)
    #     '''
    #     x = str(-self.value_X)
    #     y = self.value_Y
    #     if y > 20 or y < 20:
    #         y = str(20 - y)
    #     elif y == 20:
    #         y = str(0)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)

    # def goto_seventh(self):
    #     '''
    #     (X = 21, Y = 20)
    #     '''
    #     x = self.value_X 
    #     if x > 21 or x < 21:
    #         x = str(21 - x)
    #     elif x == 21:
    #         x = str(0)
    #     y = self.value_Y
    #     if y > 20 or y < 20:
    #         y = str(20 - y)
    #     elif y == 20:
    #         y = str(0)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)    

    # def goto_eight(self):
    #     '''
    #     (X = 42, Y = 20)
    #     '''    
    #     x = self.value_X 
    #     if x > 42 or x < 42:
    #         x = str(42 - x)
    #     elif x == 42:
    #         x = str(0)
    #     y = self.value_Y
    #     if y > 20 or y < 20:
    #         y = str(20 - y)
    #     elif y == 20:
    #         y = str(0)
    #     # print("G01 X"+x+" Y"+y)
    #     self.command("G01 X"+x+" Y"+y)   

    def draw_X(self):
        self.crosses.append([self.value_X, self.value_Y])
        self.command("G01 X5.25 Y-2.5")
        self.command("M03 S255")
        self.command("X10.5 Y-5")
        self.command("M05")
        self.command("Y5")
        self.command("M03 S255")
        self.command("X-10.5 Y-5")
        self.command("M05")
        self.command("X-5.25 Y7.5")
        
    def draw_plane(self):
        self.command("G01 Y-10")
        self.command("M03 S255")
        self.command("X65")
        self.command("M05")
        self.command("Y-10")
        self.command("M03 S255")
        self.command("X-65")
        self.command("M05")
        self.command("X21 Y20")
        self.command("M03 S255")
        self.command("Y-30")
        self.command("M05")
        self.command("X21")
        self.command("M03 S255")
        self.command("Y30")
        self.command("M05")
        self.command("X-42")