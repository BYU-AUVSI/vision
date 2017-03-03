import os
import sys
# sudo apt-get install python-tk
from Tkinter import *
# sudo apt-get install python-imaging python-imaging-tk
from PIL import Image, ImageTk
from functools import partial

# extend the tkinter class Frame to adapt to our current application
class Application(Frame):
    # method that destroys the old frame and loads a new image
    def loadNextImage(self):
    # if the autodiscard function job exists
        if self._job is not None:
            # cancel the existing job
            self.after_cancel(self._job)
            # restart the discard function to sync with the two second delay
            self._job= self.after(self.delay, self.autoDiscard)
        # creates the file name of the input image in the image directory
        filename = self.imagedir + '/image' + str(self.i)+ '.jpg'
        # try to open the file
        try:
            # if there is a valid file then do not display the wait message
            if os.stat(filename).st_size > 0:
               self.error.grid_forget()
        except OSError:
            # else display a wait for image message on the GUI
            self.error.grid(row=2,column=1)
            return
            # if there have been previous images then destroy it
            if self.i > 0:
                self.panel.destroy()
            # open a new image from the image directory
            self.image = Image.open(self.imagedir + '/image' + str(self.i)+ '.jpeg')
            # resizes the image so that all are the same size
            self.image = self.image.resize((1200, 800))
            # converts the PIL image to a tk image
            image_tk = ImageTk.PhotoImage(self.image)
            # create the label with the embedded image
            self.panel = Label(self, image=image_tk)
            # save the image as an attribute of the panel
            self.panel.image = image_tk
            # display the image in the GUI across 4 columns
            self.panel.grid(row=0, column=1, columnspan=4)
            # increment the image number
            self.i += 1

    # function that discards the current image and calls load next image on a automatic delay
    def autoDiscard(self):
        if self.image:
            # save the discarded image in a folder in case we want to go back and review them
            self.image.save(self.discarddir + '/image_' + str(self.i) +'.jpg')
        # grab the next image
        self.loadNextImage()

    # save the image to a specific target directory
    def saveImage(self, discard=False):
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

    # create the buttons for the GUI and attach the corresponding functions
    def createWidgets(self):
        # create the button that allows us to discard the image, attaches the save image function with discard always True
        self.discard= Button(self, width=20, height=1, text="DISCARD", command=partial(self.saveImage,True))
        # display the button on the GUI
        self.discard.grid(row=1, column=1)

        # create the button that allows us to save the image to a target directory, attaches the save image function with discard always False
        self.save = Button(self, width=20, height=1, text="SAVE", command=self.saveImage)
        # display the button on the GUI
        self.save.grid(row=1, column=2)

        # create the button that allows us to quit the program
        self.quit = Button(self, width=15, height=1, text="QUIT", fg="red", command=self.quit)
        # display the button on the GUI
        self.quit.grid(row=1, column=4)

        # create a label to explain the functionality of the radio buttons
        w = Label(self, text="Select Target:")
        # display the label on the GUI
        w.grid(row=1,column=3)

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

    # change the target directory to the integer associated with the radio button
    def initTarget(self):
        self.targetdir = self.targetdir[:-1] + str(self.v.get())

    # initialize the application
    def __init__(self, master=None):
        #input image counter
        self.i = 0
        # auto discard delay time in milliseconds
        self.delay = 2000
        # create the image file and initialize it to none
        self.image = None
        # read in the second argument as the number of targets
        self.targets=int(sys.argv[1])
        # create the frame
        Frame.__init__(self, master)
        # pack it up
        self.pack()

        # check to see if the output directory exists and create it if it doesn't
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
            os.makedires(self.imagedir)

        # start the auto discard function
        self._job = self.after(self.delay, self.autoDiscard)
        # create the wait for next image error
        self.error = Label(self, text='WAIT FOR NEXT IMAGE')
        # load the first image
        self.loadNextImage()
        # add all the buttons
        self.createWidgets()

# create the application and run it
root = Tk()
app = Application(master=root)
app.mainloop()
root.destroy()

