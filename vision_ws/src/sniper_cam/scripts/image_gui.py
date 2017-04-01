#! /usr/bin/env python
import os
import sys
import numpy as np
# sudo apt-get install python-tk
from Tkinter import *
import tkFileDialog
import tkMessageBox
# sudo apt-get install python-imaging python-imaging-tk
from PIL import Image as Im
from PIL import ImageTk
from functools import partial
from math import ceil, sqrt
import rospy
import roslib					
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sniper_cam.msg import interopImages

from cv_bridge import CvBridge, CvBridgeError	
       
# extend the tkinter class Frame to adapt to our current application
class Application(Frame):
    # method that destroys the old frame and loads a new image
    def loadNextImage(self):
        # if the autodiscard function job exists
        #if self.job is not None:
            # cancel the existing job
            #self.after_cancel(self.job)
            # restart the discard function to sync with the two second delay
            #self.job= self.after(self.delay, self.autoDiscard)
        # creates the file name of the input image in the image directory
        if self.rotateImage:
            self.rotateImage=None

        if self.rect:
            self.canvas.delete(self.rect)
            self.rect=None

        try:
            filename = self.imagedir  + '/' + self.images[self.i]
            # if there is a valid file then do not display the wait message
            self.error.grid_forget()
        except IndexError:
            # else display a wait for image message on the GUI
            self.error.grid(row=2,column=1)
            return
        # if there have been previous images then destroy it
        #if self.i > 0:
            #self.panel.destroy()
        # open a new image from the image directory
        self.image = Image.open(filename)
        self.originalImage = self.image
        width, height = self.image.size
        self.w_mult = float(width) / 370
        self.h_mult = float(height) / 370
        # resizes the image so that all are the same size
        self.image = self.image.resize((370, 370))
        # converts the PIL image to a tk image
        self.image_tk = ImageTk.PhotoImage(self.image)
        # create the label with the embedded image
        self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
        self.imageType = 'Standard'
        # increment the image number
        self.i += 1

    # function that discards the current image and calls load next image on a automatic delay
    def autoDiscard(self):
        self.images = os.listdir('images/')
        self.rotateValue.set(0)
        if self.image:
            # save the discarded image in a folder in case we want to go back and review them
            self.image.save(self.discarddir + '/image_' + str(self.i) +'.jpg')
        self.job= self.after(self.delay, self.autoDiscard)
        # grab the next image
        self.loadNextImage()

    # save the image to a specific target directory
    def saveImage(self, discard=False):
        self.pauseDiscard()
        self.rotateValue.set(0)
        # if the image exists
        if self.image:
                # if we are manually discarding the image from a button click
            if discard:
                self.image.save(self.discarddir + '/image_' + str(self.i) + '.jpg')
            # we are saving the image from a button click
            else:
                # grab the correct target directory based on the radio button selected
                index = self.index[int(self.targetdir[-1])-1]
                # save the image to that location
                self.image.save(self.targetdir + '/image_' + str(index) + '.jpg')
                # increment the image number in the index list (each entry corresponds to a different target directory)
                self.index[int(self.targetdir[-1])-1] = index + 1
            # grab the next image
            self.loadNextImage()

    def submitInfo(self):
	if self.imageType == 'Rotated':
		self.imageMessage = self.rotateImage
	elif self.imageType == 'Cropped':
		self.imageMessage = self.croppedImage
	else:
		self.imageMessage = self.image
	try:
	    image_msg = self.bridge.cv2_to_imgmsg(np.array(self.imageMessage), "bgr8")
	except CvBridgeError as e:
	    print(e)

        file = 'characteristics_target_{}.txt'.format(self.targetDir[-1])
        writeFile = open(file, 'wb')
        heading = self.vals[2] - self.rotateValue.get()
        if(heading < 0):
            heading += 360

	#add target type

	self.msg.gps_lati = float(self.vals[0])
	self.msg.gps_longit = float(self.vals[1])
	self.msg.target_color = self.tColorContent.get()
	self.msg.target_shape = self.tShapeContent.get()
	self.msg.symbol = self.lColorContent.get()
	self.msg.symbol_color = self.letterContent.get()
	self.msg.orientation = str(heading)

	self.pub.publish(self.msg)
	'''
        writeFile.write('{}\n,{}\n,{}\n,{}\n,{}\n,{}\n,{}\n,{}'.format(self.targetDir[-1], self.vals[0], self.vals[1], 
                                                            heading, self.tShapeContent.get(), self.letterContent.get(),
                                                            self.tColorContent.get(), self.lColorContent.get()))
	'''
        self.tColorContent.set("")
        self.tShapeContent.set("")
        self.lColorContent.set("")
        self.letterContent.set("")

    def restartDiscard(self):
        self.job = self.after(self.delay, self.autoDiscard)

    def pauseDiscard(self):
        try:
            self.after_cancel(self.rotateJob)
        except AttributeError:
            pass
        self.after_cancel(self.job)
        
        
    def sampleRotate(self):
        width, height = self.image.size
        expand = False if height > 650 else True
        if self.rotateValue.get() != self.refValue:
            self.refValue = self.rotateValue.get()
            if self.croppedImage:
                self.rotateImage = self.croppedImage.rotate(self.refValue, resample=Im.BICUBIC, expand=expand)
            else:
                self.rotateImage = self.image.rotate(self.refValue, resample=Im.BICUBIC, expand=expand)
            #self.panel.destroy()
            # converts the PIL image to a tk image
            self.image_tk = ImageTk.PhotoImage(self.rotateImage)
            # create the label with the embedded image
            self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
            self.imageType = 'Rotated'
            # display the image in the GUI across 5 columns
            #self.panel.grid(row=0, column=1, columnspan=5)
        self.rotateJob = self.after(1000, self.sampleRotate)


    def on_button_press(self, event):
        if self.rect:
            self.canvas.delete(self.rect)
        # save mouse drag start position
        self.start_x = event.x
        self.start_y = event.y
        # create rectangle if not yet exist
        #if not self.rect:
        self.rect = self.canvas.create_rectangle(self.x, self.y, 1, 1, fill="")

    def on_move_press(self, event):
        self.curX, self.curY = (event.x, event.y)
        # expand rectangle as you drag the mouse
        self.canvas.coords(self.rect, self.start_x, self.start_y, self.curX, self.curY)

    def on_button_release(self, event):
        pass

    def right_click(self, event):
        if self.rect:
            self.canvas.delete(self.rect)
            self.rect=None
    #def drawRectangle(self):
         

    def cropImage(self):
        if not self.rect:
            return

        width, height = self.image.size
        offsetX = (self.master.winfo_screenwidth()/2) - (width / 2)
        offsetY = ((self.master.winfo_screenheight()-200)/2) - (height / 2)
        self.toBeCropped = self.originalImage

        if self.imageType=='Cropped':
            self.toBeCropped = self.croppedImage
        
        if self.imageType=='Rotated':
            rWidth, rHeight = self.rotateImage.size
            if rWidth != width:
                offsetX = offsetX - (rWidth - width) / 2
                offsetY = offsetY - (rHeight - height) / 2
                self.toBeCropped = self.rotateImage
            else:
                self.toBeCropped = self.image           

        if self.image:
           # print(self.toBeCropped.size)
            #print('{0} {1}'.format(offsetX, offsetY))
            if self.imageType == 'Standard':
                self.croppedImage = self.toBeCropped.crop((int((self.start_x-offsetX)*self.w_mult), int((self.start_y-offsetY)*self.h_mult),
                                    int((self.curX-offsetX)*self.w_mult), int((self.curY-offsetY)*self.h_mult)))
            else:
                self.croppedImage = self.toBeCropped.crop((self.start_x-offsetX, self.start_y-offsetY,self.curX-offsetX, self.curY-offsetY))
            self.croppedImage = self.croppedImage.resize((370,370))
            #print('{0} {1} {2} {3}'.format(self.start_x, self.start_y, self.curX, self.curY))
            self.image_tk = ImageTk.PhotoImage(self.croppedImage)
            # create the label with the embedded image
            self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
            self.imageType = 'Cropped'
            self.canvas.delete(self.rect)

    def undoCrop(self):
        if self.croppedImage:
            self.rotateImage=None
            self.croppedImage=None
            self.rotateValue.set(0)
            self.image_tk = ImageTk.PhotoImage(self.image)
            # create the label with the embedded image
            self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
            self.imageType = 'Standard'

    # create the buttons for the GUI and attach the corresponding functions
    def createWidgets(self):
        self.cropButton = Button(self, width=10, height=1, text="CROP", command=self.cropImage, state=DISABLED)
        self.cropButton.grid(row=1, column=6)
        self.undo = Button(self, width=10, heigh=1, text="UNDO CROP", command=self.undoCrop, state=DISABLED)
        self.undo.grid(row=2, column=6)
        #self.select = Button(self, width=10, height=1, text="SELECT", command=self.drawRectangle)
        #self.select.grid(row=1, column=6)
        # create the button that allows us to submit the information
        self.submit = Button(self, width=20, height=1, text="SUBMIT", fg="green", command=self.submitInfo, state=DISABLED)
        self.submit.grid(row=1, column=4, columnspan=2)

        self.rotateValue = IntVar()
        self.rotateValue.set(0)
        self.refValue = 0
        self.rotateLabel = Label(self, text="Counter Clockwise >>")
        self.rotateLabel.grid(row=2, column=4, columnspan=2)
        self.rotateScale = Scale(self, from_=0, to=360, orient=HORIZONTAL, width=10, length=150, sliderlength=15, variable=self.rotateValue, state=DISABLED)
        self.rotateScale.grid(row=3, column=4, columnspan=2)

        #self.rotateImage = self.rotateImage.resize(10,10)
        #self.rotateButton = Button(self, text="START ROTATING", width=15, height=1, command=self.sampleRotate, state=DISABLED)
        #self.rotateButton.grid(row=4, column=4, columnspan=2)

        #self.start = Button(self, width=9, height=1, text="START", command=self.restartDiscard)
        #self.start.grid(row=1, column=4, sticky=E)
        #self.pause = Button(self, width=8, height=1, text="PAUSE", command=self.pauseDiscard)
        #self.pause.grid(row=1, column=5, sticky=W)
        #self.after_cancel(self.job)
        # create the button that allows us to discard the image, attaches the save image function with discard always True
        #self.discard= Button(self, width=20, height=1, text="DISCARD", command=partial(self.saveImage,True))
        # display the button on the GUI
        #self.discard.grid(row=1, column=1)

        # create the button that allows us to save the image to a target directory, attaches the save image function with discard always False
        #self.save = Button(self, width=20, height=1, text="SAVE", command=self.saveImage)
        # display the button on the GUI
        #self.save.grid(row=1, column=2)

        # create the button that allows us to quit the program
        self.quit = Button(self, width=10, height=1, text="QUIT", fg="red", command=self.quit)
        # display the button on the GUI
        self.quit.grid(row=4, column=6)

        self.tColorLabel = Label(self, text="Target Color")
        self.tColorLabel.grid(row=1,column=1)

        self.tColorContent = StringVar()
        self.targetColor = Entry(self, width=20, textvariable=self.tColorContent, state=DISABLED)
        self.targetColor.grid(row=2,column=1)

        self.tShapeLabel = Label(self, text="Target Shape")
        self.tShapeLabel.grid(row=3,column=1)

        self.tShapeContent = StringVar()
        self.targetShape = Entry(self, width=20, textvariable=self.tShapeContent, state=DISABLED)
        self.targetShape.grid(row=4,column=1)

        self.lColorLabel = Label(self, text="Letter Color")
        self.lColorLabel.grid(row=1,column=2)

        self.lColorContent = StringVar()
        self.letterColor = Entry(self, width=20, textvariable=self.lColorContent, state=DISABLED)
        self.letterColor.grid(row=2,column=2)

        self.letterLabel = Label(self, text="Letter")
        self.letterLabel.grid(row=3,column=2)

        self.letterContent = StringVar()
        self.letter = Entry(self, width=20, textvariable=self.letterContent, state=DISABLED)
        self.letter.grid(row=4,column=2)

        # create a label to explain the functionality of the radio buttons
        self.targetLabel = Label(self, text="Select Target Directory")
        # display the label on the GUI
        self.targetLabel.grid(row=1,column=3)

        #self.targetContent = StringVar()
        #self.target = Entry(self, width=20, textvariable=self.targetContent)
        #self.target.grid(row=2,column=3)

        self.targetButton = Button(self, width=18, height=1, text="BROWSE", command=self.browseDirectory)
        self.targetButton.grid(row=2, column=3)
        """
        # create a integer object to pass to the radio button
        self.v = IntVar()
        # initialize its value to 1
        self.v.set(1)
        # create the list that tracks how many images have been saved in each target directory
        self.index = []
        # create a radio button for each of the targets
        
        for i in range(1,self.targets+1):
            # add an entry in the image number list
            self.index.append(0)
            # create the radio button and add the initialize target as the function, attach the integer object to track which button is selected
            Radiobutton(self, text="Target " + str(i), variable=self.v, value=i, command=self.initTarget).grid(row=i+1, column=3)
            # create the target directory name associated with the button
            self.targetdir = os.path.join(self.outputdir, 'target' + str(i))
            # if the directory doesn't already exist
            if not os.path.isdir(self.targetdir):
                # create the directory
                os.makedirs(self.targetdir)
        # set the target directory to be one as a default when the GUI starts 
        self.targetdir = self.targetdir[:-1] + '1'
        """
    def loadImage(self, event):
        if self.red_rect:
            self.canvas.delete(self.red_rect)

        filename='{}/{}'.format(self.targetDir,self.images[int(self.red_pos[0]*self.columns+self.red_pos[1])])
        try:
            self.image = Im.open(filename)     
        except OSError:
            return
        self.canvas.unbind('<Button 1>')
        self.master.unbind('<Up>')
        self.master.unbind("<Down>")
        self.master.unbind("<Left>")
        self.master.unbind("<Right>")

        self.sampleRotate()
        self.savedImages=[]
        self.canvas.delete(self.red_rect)
        self.submit.configure(state=NORMAL)
        self.cropButton.configure(state=NORMAL)
        self.undo.configure(state=NORMAL)
        self.rotateScale.configure(state=NORMAL)
        self.targetShape.configure(state=NORMAL)
        self.targetColor.configure(state=NORMAL)
        self.letter.configure(state=NORMAL)
        self.letterColor.configure(state=NORMAL)
        self.canvas.bind("<ButtonPress-1>",self.on_button_press)
        self.canvas.bind("<ButtonPress-3>", self.right_click)
        self.canvas.bind("<B1-Motion>", self.on_move_press)
        self.canvas.bind("<ButtonRelease-1>", self.on_button_release) 
        self.image = Im.open(filename)
        self.originalImage = self.image
        width, height = self.image.size
        self.w_mult = float(width) / 370
        self.h_mult = float(height) / 370
        # resizes the image so that all are the same size
        self.image = self.image.resize((370, 370), Im.ANTIALIAS)
        # converts the PIL image to a tk image
        self.image_tk = ImageTk.PhotoImage(self.image)
        # create the label with the embedded image
        self.canvas.create_image(self.master.winfo_screenwidth()/2,(self.master.winfo_screenheight()-200)/2,anchor=CENTER, image=self.image_tk)
        self.imageType = 'Standard'

    def browseDirectory(self):
        self.targetDir = tkFileDialog.askdirectory()
        self.loadFiles()

    def averagePositionVals(self):
        self.vals = [0,0]
        values = open('{}/target_{}_locations.txt'.format(self.paramDir, self.targetDir[-1]), 'rb')
	count = 0
        for line in values:
	    if(line!='\n'):
		split_vals = line.split(',')
		self.vals[0] += float(split_vals[2])
		self.vals[1] += float(split_vals[3])
		count += 1
	    else:
		break

        self.vals = list(map(lambda x: x / count, self.vals))
	self.vals.append(200)


    def loadFiles(self, event=None):
        if self.image:
            self.image_tk=None

	#print(self.targetDir)
        # sample the directory for images
        try:
            files = os.listdir(self.targetDir)
        except OSError:
            return
        self.cropButton.configure(state=DISABLED)
        self.undo.configure(state=DISABLED)
        self.rotateScale.configure(state=DISABLED)
        self.targetShape.configure(state=DISABLED)
        self.targetColor.configure(state=DISABLED)
        self.letter.configure(state=DISABLED)
        self.letterColor.configure(state=DISABLED)

        self.canvas.bind("<Button-1>", self.move_red_rect)
        self.canvas.bind("<Double-Button-1>", self.loadImage)
        self.canvas.bind("<Button-2>", self.loadFiles)
        self.master.bind("<Up>", self.move_up)
        self.master.bind("<Down>", self.move_down)
        self.master.bind("<Left>", self.move_left)
        self.master.bind("<Right>", self.move_right)

        self.images = [x for x in files if '.jpg' in x]
        self.savedImages = []
	self.paramDir = os.path.join(os.path.dirname(os.path.dirname(self.targetDir)),'target_locations')
	try:
            locations_files = os.listdir(self.paramDir)
        except OSError:
            return
	#print(locations_files)
        self.averagePositionVals()
        # divide the images into equal sizes
        self.columns = ceil(sqrt(len(self.images)))
        self.rows = ceil(len(self.images) / self.columns)
        width=self.master.winfo_screenwidth()
        height=self.master.winfo_screenheight()-200
        j=0
        k=0
        self.column_size = int(width/self.columns)
        self.row_size = int(height/self.rows)
        for i in range(len(self.images)):
	    print(self.images[i])
            image = Im.open('{}/{}'.format(self.targetDir,self.images[i]))
            image = image.resize((self.column_size, self.row_size))
            # converts the PIL image to a tk image
            image_tk = ImageTk.PhotoImage(image)
            self.savedImages.append(image_tk)
            self.canvas.create_image(j*(self.column_size)+self.column_size/2,k*self.row_size+self.row_size/2, image=image_tk)
            j+=1
            if(j==self.columns):
                j=0
                k+=1
        self.red_rect = self.canvas.create_rectangle(0, 0, self.column_size, self.row_size, fill='',outline='red')
        self.red_pos = (0,0)

    def move_up(self, event):
        if not self.red_rect:
            return
        if self.red_pos[0] == 0:
            if(((self.rows-1)*self.columns+self.red_pos[1]) > (len(self.images)-1)):
                self.red_pos = (self.rows-2, self.red_pos[1])
            else:
                self.red_pos = (self.rows-1, self.red_pos[1])
        else:
            self.red_pos = (self.red_pos[0]-1, self.red_pos[1])
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size, 
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')
    def move_down(self, event):
        if not self.red_rect:
            return
        if self.red_pos[0] == self.rows-1:
            self.red_pos = (0, self.red_pos[1])
        else:
            if(((self.red_pos[0]+1)*self.columns+self.red_pos[1]) > (len(self.images)-1)):
                self.red_pos = (0, self.red_pos[1])
            else:
                self.red_pos = (self.red_pos[0]+1, self.red_pos[1])
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size, 
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')

    def move_left(self, event):
        if not self.red_rect:
            return
        if self.red_pos[1] == 0:
            if((self.red_pos[0]*self.columns+(self.columns-1)) > (len(self.images)-1)):
                self.red_pos = (self.red_pos[0], (len(self.images) % self.rows)-1) 
            else:
                self.red_pos = (self.red_pos[0], self.columns-1)
        else:
            self.red_pos = (self.red_pos[0], self.red_pos[1]-1)
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size, 
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')
    def move_right(self, event):
        if not self.red_rect:
            return
        if self.red_pos[1] == self.columns-1:
            self.red_pos = (self.red_pos[0], 0)
        else:
            if(self.red_pos[0]*self.columns+self.red_pos[1]+1) > (len(self.images)-1):
                self.red_pos = (self.red_pos[0], 0);
            else:
                self.red_pos = (self.red_pos[0], self.red_pos[1]+1)
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size, 
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')
    
    def move_red_rect(self, event):
        if not self.red_rect:
            return
        column = int(event.x / self.column_size)
        row = (event.y / self.row_size)
        if((row*self.columns+column) > (len(self.images)-1)):
            return
        self.red_pos = (row, column)
        self.canvas.delete(self.red_rect)
        self.red_rect = self.canvas.create_rectangle(self.red_pos[1]*(self.column_size), self.red_pos[0]*self.row_size, 
                                                     self.red_pos[1]*(self.column_size) + self.column_size, self.red_pos[0]*self.row_size+self.row_size,
                                                     fill='',outline='red')

    # initialize the application
    def __init__(self, master=None):
        self.master = master
        #self.images = os.listdir('images/')
        #input image counter
        self.i = 0
        # auto discard delay time in milliseconds
        self.delay = 2000
        # create the image file and initialize it to none
        self.image = None
        self.displayImage = None
        self.rotateImage = None
        self.croppedImage = None
        self.imageType = ''
	
	self.pub = rospy.Publisher('plans', interopImages, queue_size =  10) 	
	self.bridge = CvBridge()
	self.msg = interopImages()

	rospy.init_node('death_star', anonymous=True)
	#rate = rospy.Rate(1)

        # create the frame
        Frame.__init__(self, master)
        # pack it up
        self.pack()

        # check to see if the output directory exists and create it if it doesn't
        '''
        self.outputdir = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'output')
        if not os.path.isdir(self.outputdir):
            os.makedirs(self.outputdir)
        # check to see if the discard directory exists and create it if it doesn't
        self.discarddir = os.path.join(self.outputdir, 'discarded')
        if not os.path.isdir(self.discarddir):
            os.makedirs(self.discarddir)
        # check to see if the image directory exists and create it if it doesn't
        self.imagedir = os.path.join(os.path.dirname(
            os.path.realpath(__file__)), 'images')
        if not os.path.isdir(self.imagedir):
            os.makedirs(self.imagedir)
            '''
        self.x = self.y = 0
        self.canvas = Canvas(self, height=self.master.winfo_screenheight()-200, width=self.master.winfo_screenwidth(), cursor="cross")
        self.canvas.grid(row=0, column=1, columnspan=6)
        self.rect=None
        # start the auto discard function
        #self.job = self.after(self.delay, self.autoDiscard)
        # create the wait for next image error
        # self.error = Label(self, text='WAIT FOR NEXT IMAGE')
        # load the first image
        #self.loadNextImage()
        # add all the buttons
        self.createWidgets()
        self.master.grab_set()
        self.master.grab_release()
        #tkMessageBox.showinfo(
         #   "Instructions",
          #  "Open target directory to load images and ROS data")
        #tkMessageBox.showinfo(
         #   "Instructions",
          #  "1. Click an image to highlight\n2. Double click an image to select\n3. Middle click to undo")

# create the application and run it
root = Tk()
#root.resizable(width=False, height=False)
width, height = root.winfo_screenwidth(), root.winfo_screenheight()
app = Application(master=root)
app.mainloop()
root.destroy()

