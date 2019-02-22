#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_collision_detection')
import rospy
import rospkg
from robot_collision_detection.msg import CollPart
from robot_collision_detection.srv import GetParts, GetPartsRequest, GetPartsResponse

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TwistStamped, WrenchStamped, Vector3, PointStamped, Point
from qb_interface.msg import handRef, handPos
from std_msgs.msg import UInt32MultiArray, String
from std_srvs.srv import Empty, EmptyRequest, Trigger, TriggerRequest
import time
import matplotlib as mpl
import numpy as np
import sys
import os
from subprocess import call
if sys.version_info[0] < 3:
    import Tkinter as tk
else:
    import tkinter as tk
import matplotlib.backends.tkagg as tkagg
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from IPython import embed
#parts=GetParts()

window = tk.Tk()


class PartUI():
    def __init__(self, *args, **kwargs):
        rospy.init_node('analyse_parts_ui', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.is_running=True;
        self.n_links=8;
        rospy.wait_for_service('robot_collision_detection/step_estimation')
        self.srv_client = rospy.ServiceProxy('robot_collision_detection/step_estimation', Empty)
        self.srv_client2 = rospy.ServiceProxy('robot_collision_detection/pause_estimation', Trigger)
        self.srv_restart = rospy.ServiceProxy('robot_collision_detection/restart_estimation', Empty)


        self.p_sub=rospy.Subscriber("/contact_point_local", PointStamped, self.p_callback)
        self.f_sub=rospy.Subscriber("/contact_force", WrenchStamped, self.f_callback)

        self.start_canvas()

        self.create_widgets(window)
        self.get_plot_ranges()
        self.p_true=Point()
        self.f_true=Vector3()

    def load_param_file(self):
    	file='/config/config_collision_iiwa.yaml'
    	#embed()
        #folder=subprocess.check_output('rospack find robot_collision_detection')
        #folder=call('rospack find robot_collision_detection')
        folder=rospkg.RosPack().get_path('robot_collision_detection')
        command='rosparam load '+folder+file
        print(command)
        os.system('ROS_NAMESPACE=robot_collision_detection ' + command)


    def create_widgets(self,parent=window):
        b = tk.Button(parent, text="Pause", command=self.pause_callback)
        b.grid(row=0,column=1)
        b = tk.Button(parent, text="Embed", command=self.interrupt_callback)
        b.grid(row=1,column=1)
        b = tk.Button(parent, text="step", command=self.callStep)
        b.grid(row=1,column=2)
        b = tk.Button(parent, text="resize", command=self.get_plot_ranges)
        b.grid(row=0,column=2)
        b = tk.Button(parent, text="restart", command=self.restart_estimation)
        b.grid(row=0,column=3)


        r_group = tk.LabelFrame(parent, text="Show", padx=5, pady=5)
        r_group.grid(row=2,column=1,rowspan = 1)

        self.v_radio=tk.IntVar(value=1)
        r = tk.Radiobutton(r_group, text="Forces",variable=self.v_radio, value=1)
        r.grid(row=0,column=0)        
        r = tk.Radiobutton(r_group, text="Stiffness",variable=self.v_radio, value=2)
        r.grid(row=1,column=0)




        group = tk.LabelFrame(parent, text="Display Links", padx=5, pady=5)
        group.grid(row=3,column=1)
        #group.pack(padx=10, pady=10)

        self.check_all = tk.IntVar(value=1)
        c = tk.Checkbutton(group, text="enable All", variable=self.check_all,command=self.enable_all_links_callback)
        c.pack()
        self.checks=[tk.IntVar(value=1) for i in range(self.n_links)]
        for i in range(self.n_links):
            c = tk.Checkbutton(group, text="link"+str(i), command=self.check_changed_cb,variable=self.checks[i])        
            c.pack()


    def f_callback(self,msg):
        self.f_true=msg.wrench.force;

    def p_callback(self,msg):
        self.p_true=msg.point


    def check_changed_cb(self):
        self.check_all.set(0)

    def interrupt_callback(self):
        parts=self.getParts()
        x,y,n,w=self.parts_to_xy(parts.part)
           

    def restart_estimation(self):
    	self.load_param_file()
        self.srv_restart(EmptyRequest())


    def pause_callback(self):
        print "paused!"
        self.load_param_file()
        self.srv_client2(TriggerRequest())
        self.is_running = not self.is_running

    def enable_all_links_callback(self):        
        for i in range(len(self.checks)):
            self.checks[i].set(self.check_all.get())


    def get_plot_ranges(self):
        parts=self.getParts()
        x,y,n,w=self.parts_to_xy(parts.part)
        x,y,n,w=self.select_links(x,y,n,w)        
        self.xmin=min(x)
        self.xmax=max(x)
        self.ymin=min(y)
        self.ymax=max(y)

    def w_to_colors(self,ws,max_wa=0):
        c=np.empty((0,3))
        if(len(ws)>0):
            if(max_wa==0):
                max_wa=max(ws)
            min_w=min(ws)
            max_w=max_wa-min_w
            for i in range(0,len(ws)):
                rw=(ws[i]-min_w)/max_w
                c=np.vstack([c,[(1-rw)**5,rw**5,0]])
        return c

    def onclick(self,event):
        #print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %('double' if event.dblclick else 'single', event.button,event.x, event.y, event.xdata, event.ydata))
        rospy.loginfo("FUUUUU")


    def start_canvas(self):
        # Create a canvas
        w, h = 800,800
        window.title("A figure in a canvas")
        self.canvas = tk.Canvas(window, width=w, height=h)
        #self.canvas.pack()
        self.canvas.grid(row=0,column=0,rowspan = 5)

        # Create the figure we desire to add to an existing canvas
        self.fig = mpl.figure.Figure(figsize=(10, 8))        
        self.ax = self.fig.add_axes([0.08, 0.08, 0.9, 0.9],picker=10)
        

        # Keep this handle alive, or else figure will disappear
        self.fig_x, self.fig_y = 10, 10
        self.fig_photo = self.draw_figure(self.canvas, self.fig, loc=(self.fig_x, self.fig_y))
        fig_w, fig_h = self.fig_photo.width(), self.fig_photo.height()
        self.cid3 = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

        # Add more elements to the canvas, potentially on top of the figure
        #canvas.create_line(200, 50, fig_x + fig_w / 2, fig_y + fig_h / 2)
        #canvas.create_text(200, 50, text="Zero-crossing", anchor="s")
        
        

        # Let Tk take over
        #window.mainloop()


    def getParts(self):
        rospy.wait_for_service('robot_collision_detection/get_particles')
        try:
            srv_client = rospy.ServiceProxy('robot_collision_detection/get_particles', GetParts)
            parts = srv_client(GetPartsRequest())
            return parts
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def callStep(self):
    	#self.is_running=False
        try:
            self.load_param_file()
            self.srv_client(EmptyRequest())
            self.redraw()
            window.update()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def toVec(self,v_in):
        return np.array([v_in.x,v_in.y,v_in.z])

    def parts_to_xy(self,parts):
        x=[]    
        y=[]
        w=[]
        n=[]        
        for p in parts:
            n.append(p.n)
            x.append(np.linalg.norm(self.toVec(p.p)))
            #x.append(p.p.z)
            if(self.v_radio.get()==1):
                y.append(np.linalg.norm(self.toVec(p.F.force)))
            elif(self.v_radio.get()==2):            
                y.append(p.K)
            w.append(p.w)
        return np.array(x),np.array(y),np.array(n),np.array(w)

    def select_links(self,x,y,n,w):
        x_out=np.array([])
        y_out=np.array([])
        n_out=np.array([])
        w_out=np.array([])

        for i in range(len(self.checks)):            
            if (self.checks[i].get()==1):
                n_out=np.append(n_out,n[np.where(n==i)])
                x_out=np.append(x_out,x[np.where(n==i)])
                y_out=np.append(y_out,y[np.where(n==i)])
                w_out=np.append(w_out,w[np.where(n==i)])              
        return x_out,y_out,n_out,w_out

    def onpick3(event):
        ind = event.ind
        print('onpick3 scatter:', ind, np.take(x, ind), np.take(y, ind))



    def redraw(self):
        parts=self.getParts()
        x,y,n,w=self.parts_to_xy(parts.part)            
        ma=max(w)
        x,y,n,w=self.select_links(x,y,n,w)        
        c=self.w_to_colors(w,max_wa=ma)

        x=np.append(x,np.linalg.norm([self.p_true.x,self.p_true.y,self.p_true.z]))
        #x=np.append(x,self.p_true.z)
        if(self.v_radio.get()==1):
            y=np.append(y,np.linalg.norm([self.f_true.x,self.f_true.y,self.f_true.z]))
        elif(self.v_radio.get()==2):
            y=np.append(y,5000)
        c=np.vstack([c,[0,0,1]])

        #print(str(x[-1]) + " -- "+ str(y[-1]))
        self.ax.cla()
        ss=[50]*len(x);
        ss[-1]=100;
        #embed()
        plt=self.ax.scatter(x,y,s=ss,c=c,picker=10)
        self.cid10=plt.figure.canvas.mpl_connect('pick_event', self.onpick3)
        self.cid11 = plt.figure.canvas.mpl_connect('button_press_event', self.onclick)

        
        self.ax.set_xlim(self.xmin,self.xmax)
        if(self.v_radio.get()==1):
            self.ax.set_ylim(self.ymin,min(200,self.ymax))
        elif(self.v_radio.get()==2):
            self.ax.set_ylim(self.ymin,self.ymax)
        
        self.ax.set_xlabel('l')
        if(self.v_radio.get()==1):
            self.ax.set_ylabel('F')
        elif(self.v_radio.get()==2):            
            self.ax.set_ylabel('K')

        self.fig_photo = self.draw_figure(self.canvas, self.fig, loc=(self.fig_x, self.fig_y))
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            #if self.is_running:
            self.redraw()
            	#self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
            window.update()



    def draw_figure(self,canvas, figure, loc=(0, 0)):
        """ Draw a matplotlib figure onto a Tk canvas

        loc: location of top-left corner of figure on canvas in pixels.
        Inspired by matplotlib source: lib/matplotlib/backends/backend_tkagg.py
        """
        figure_canvas_agg = FigureCanvasAgg(figure)
        self.cid = figure_canvas_agg.mpl_connect('button_press_event', self.onclick)

        #embed()



        figure_canvas_agg.draw()
        figure_x, figure_y, figure_w, figure_h = figure.bbox.bounds
        figure_w, figure_h = int(figure_w), int(figure_h)
        photo = tk.PhotoImage(master=canvas, width=figure_w, height=figure_h)    

        # Position: convert from top-left anchor to center anchor
        canvas.create_image(loc[0] + figure_w/2, loc[1] + figure_h/2, image=photo)

        # Unfortunately, there's no accessor for the pointer to the native renderer
        tkagg.blit(photo, figure_canvas_agg.get_renderer()._renderer, colormode=2)

        #embed()

        # Return a handle which contains a reference to the photo object
        # which must be kept live or else the picture disappears
        return photo

ui=PartUI()
ui.run()