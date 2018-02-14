#!/usr/bin/env python2.7
import gym
import reacher.Reacher
import numpy as np
import cv2
import math
import scipy as sp
import collections
import time
class MainReacher():
    def __init__(self):
        self.env = gym.make('ReacherMy-v0')
        self.env.reset()

    def detect_l1(self,image,quadrant):
        #In this method you can focus on detecting the rotation of link 1, colour:(102,102,102)
        #Obtain the center of link 1
        mask = cv2.inRange(image, (101,101,101),(104,104,104))
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = np.array([cx,cy])
        #apply the distance transform
        dist = cv2.distanceTransform(cv2.bitwise_not(mask),cv2.DIST_L2,0)
        sumlist = np.array([])
        #step is how the degree increment to step through in the search
        step = 0.5
        if quadrant == "UR":
            #should be between 0 and math.pi/2
            for i in np.arange(0,90,step):
                #Rotate the template to the desired rotation configuration
                M = cv2.getRotationMatrix2D((self.env.rod_template_1.shape[0]/2,self.env.rod_template_1.shape[1]/2),i,1)
                #Isolate the region of interest in the distance image
                ROI = dist[(cy-self.env.rod_template_1.shape[0]/2):(cy+self.env.rod_template_1.shape[0]/2)+1,(cx-self.env.rod_template_1.shape[1]/2):(cx+self.env.rod_template_1.shape[1]/2)+1]
                #Apply rotation to the template
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_1,M,self.env.rod_template_1.shape)
                #Combine the template and region of interest together to obtain only the values that are inside the template
                img = ROI*rotatedTemplate
                #Sum the distances and append to the list
                sumlist = np.append(sumlist,np.sum(img))
            #Once all configurations have been searched then select the one with the smallest distance and convert to radians.
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*step))*math.pi)/180.0)
        elif quadrant == "UL":
            #should be between math.pi/2 and math.pi (REPEAT THE SAME AS ABOVE JUST WITH DIFFERENT LIMITS)
            for i in np.arange(90+step,180,step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_1.shape[0]/2,self.env.rod_template_1.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_1.shape[0]/2):(cy+self.env.rod_template_1.shape[0]/2)+1,(cx-self.env.rod_template_1.shape[1]/2):(cx+self.env.rod_template_1.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_1,M,self.env.rod_template_1.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*step)+90+step)*math.pi)/180.0)
        elif quadrant == "LR":
            #should be between -0 and -math.pi/2
            for i in np.arange(-step,-90,-step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_1.shape[0]/2,self.env.rod_template_1.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_1.shape[0]/2):(cy+self.env.rod_template_1.shape[0]/2)+1,(cx-self.env.rod_template_1.shape[1]/2):(cx+self.env.rod_template_1.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_1,M,self.env.rod_template_1.shape)

                img = ROI*rotatedTemplate
                #cv2.imshow('tmp',img)
                #cv2.waitKey(5)
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*-step)-step)*math.pi)/180.0)
        elif quadrant == "LL":
            #should be between -math.pi/2 and -math.pi
            for i in np.arange(-90-step,-179.5,-step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_1.shape[0]/2,self.env.rod_template_1.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_1.shape[0]/2):(cy+self.env.rod_template_1.shape[0]/2)+1,(cx-self.env.rod_template_1.shape[1]/2):(cx+self.env.rod_template_1.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_1,M,self.env.rod_template_1.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*-step)-90-step)*math.pi)/180.0)

    def detect_l2(self,image,quadrant):
        #In this method you can focus on detecting the rotation of link 2, colour:(51,51,51)
        #SAME AS ABOVE METHOD JUST WITH DIFFERENT COLOUR LIMITS
        mask = cv2.inRange(image, (50,50,50),(52,52,52))
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = np.array([cx,cy])
        dist = cv2.distanceTransform(cv2.bitwise_not(mask),cv2.DIST_L2,0)
        sumlist = np.array([])
        step = 0.5
        if quadrant == "UR":
            #should be between 0 and math.pi/2
            for i in np.arange(0,90,step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_2.shape[0]/2,self.env.rod_template_2.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_2.shape[0]/2):(cy+self.env.rod_template_2.shape[0]/2)+1,(cx-self.env.rod_template_2.shape[1]/2):(cx+self.env.rod_template_2.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_2,M,self.env.rod_template_2.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*step))*math.pi)/180.0)
        elif quadrant == "UL":
            #should be between math.pi/2 and math.pi
            for i in np.arange(90+step,180,step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_2.shape[0]/2,self.env.rod_template_2.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_2.shape[0]/2):(cy+self.env.rod_template_2.shape[0]/2)+1,(cx-self.env.rod_template_2.shape[1]/2):(cx+self.env.rod_template_2.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_2,M,self.env.rod_template_2.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*step)+90+step)*math.pi)/180.0)
        elif quadrant == "LR":
            #should be between -0 and -math.pi/2
            for i in np.arange(-step,-90,-step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_2.shape[0]/2,self.env.rod_template_2.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_2.shape[0]/2):(cy+self.env.rod_template_2.shape[0]/2)+1,(cx-self.env.rod_template_2.shape[1]/2):(cx+self.env.rod_template_2.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_2,M,self.env.rod_template_2.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*-step)-step)*math.pi)/180.0)
        elif quadrant == "LL":
            #should be between -math.pi/2 and -math.pi
            for i in np.arange(-90-step,-179.5,-step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_2.shape[0]/2,self.env.rod_template_2.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_2.shape[0]/2):(cy+self.env.rod_template_2.shape[0]/2)+1,(cx-self.env.rod_template_2.shape[1]/2):(cx+self.env.rod_template_2.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_2,M,self.env.rod_template_2.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*-step)-90-step)*math.pi)/180.0)

    def detect_l3(self,image,quadrant):
        #In this method you can focus on detecting the rotation of link 3, colour:(0,0,0)
        #SAME AS DETECT_L1 METHOD JUST WITH DIFFERENT COLOUR LIMITS
        mask = cv2.inRange(image, (0,0,0),(2,2,2))
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        center = np.array([cx,cy])
        dist = cv2.distanceTransform(cv2.bitwise_not(mask),cv2.DIST_L2,0)
        sumlist = np.array([])
        step = 0.5
        if quadrant == "UR":
            #should be between 0 and math.pi/2
            for i in np.arange(0,90,step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_3.shape[0]/2,self.env.rod_template_3.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_3.shape[0]/2):(cy+self.env.rod_template_3.shape[0]/2)+1,(cx-self.env.rod_template_3.shape[1]/2):(cx+self.env.rod_template_3.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_3,M,self.env.rod_template_3.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*step))*math.pi)/180.0)
        elif quadrant == "UL":
            #should be between math.pi/2 and math.pi
            for i in np.arange(90+step,180,step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_3.shape[0]/2,self.env.rod_template_3.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_3.shape[0]/2):(cy+self.env.rod_template_3.shape[0]/2)+1,(cx-self.env.rod_template_3.shape[1]/2):(cx+self.env.rod_template_3.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_3,M,self.env.rod_template_3.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*step)+90+step)*math.pi)/180.0)
        elif quadrant == "LR":
            #should be between -0 and -math.pi/2
            for i in np.arange(-step,-90,-step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_3.shape[0]/2,self.env.rod_template_3.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_3.shape[0]/2):(cy+self.env.rod_template_3.shape[0]/2)+1,(cx-self.env.rod_template_3.shape[1]/2):(cx+self.env.rod_template_3.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_3,M,self.env.rod_template_3.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*-step)-step)*math.pi)/180.0)
        elif quadrant == "LL":
            #should be between -math.pi/2 and -math.pi
            for i in np.arange(-90-step,-179.5,-step):
                M = cv2.getRotationMatrix2D((self.env.rod_template_3.shape[0]/2,self.env.rod_template_3.shape[1]/2),i,1)
                ROI = dist[(cy-self.env.rod_template_3.shape[0]/2):(cy+self.env.rod_template_3.shape[0]/2)+1,(cx-self.env.rod_template_3.shape[1]/2):(cx+self.env.rod_template_3.shape[1]/2)+1]
                rotatedTemplate = cv2.warpAffine(self.env.rod_template_3,M,self.env.rod_template_3.shape)

                img = ROI*rotatedTemplate
                sumlist = np.append(sumlist,np.sum(img))
            return (self.coordinate_convert(center),(((np.argmin(sumlist)*-step)-90-step)*math.pi)/180.0)


    def angle_normalize(self,x):
        #Normalizes the angle between pi and -pi
        return (((x+np.pi) % (2*np.pi)) - np.pi)


    def detect_blue(self,image):
        #In this method you can focus on detecting the center of the blue circle
        #Isolate the blue colour in the image as a binary image
        mask = cv2.inRange(image, (0,0,245),(0,0,255))
        #This applies a dilate that basically makes the binary region larger (the more iterations the larger it becomes)
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        #Obtain the moments of the binary image
        M = cv2.moments(mask)
        #Calculate pixel coordinates for the center of the blob
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #Convert pixel coordinates to world coordinates
        return self.coordinate_convert(np.array([cx,cy]))

    def detect_green(self,image):
        #In this method you should focus on detecting the center of the green circle
        #SAME AS DETECT_BLUE JUST WITH DIFFERENT COLOUR LIMITS
        mask = cv2.inRange(image, (0,245,0),(0,255,0))
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        return self.coordinate_convert(np.array([cx,cy]))

    def detect_red(self,image):
        #In this method you should focus on detecting the center of the red circle
        #SAME AS DETECT_BLUE JUST WITH DIFFERENT COLOUR LIMITS
        mask = cv2.inRange(image, (245,0,0),(255,0,0))
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        return self.coordinate_convert(np.array([cx,cy]))

    def detect_target(self,image):
        #Detect the center of the target circle (Colour: [200,200,200])
        #SAME AS DETECT_BLUE JUST WITH DIFFERENT COLOUR LIMITS
        mask = cv2.inRange(image, (177,177,177),(179,179,179))
        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations=3)
        M = cv2.moments(mask)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        return self.coordinate_convert(np.array([cx,cy]))

    def detect_joint_angles(self,image):
        #Calculate the relevant joint angles from the image
        #Obtain the center of each coloured blob
        jointPos3 = self.detect_blue(image)
        jointPos2 = self.detect_green(image)
        jointPos1 = self.detect_red(image)
        #Solve using trigonometry
        ja1 = math.atan2(jointPos1[1],jointPos1[0])
        ja2 = math.atan2(jointPos2[1]-jointPos1[1],jointPos2[0]-jointPos1[0])-ja1
        ja2 = self.angle_normalize(ja2)
        ja3 = math.atan2(jointPos3[1]-jointPos2[1],jointPos3[0]-jointPos2[0])-ja2-ja1
        ja3 = self.angle_normalize(ja3)
        return np.array([ja1,ja2,ja3])


    def detect_joint_angles_chamfer(self,image):
        #Calculate the relevant joint angles from the image
        #Obtain the center of each coloured blob
        jointPos3 = self.detect_blue(image)
        jointPos2 = self.detect_green(image)
        jointPos1 = self.detect_red(image)
        l1_cen = np.zeros(2)
        l1_angle = 0
        #Determine which quadrant link 1 is pointing in and detect the angle and center
        if jointPos1[0]>=0 and jointPos1[1] >=0:
            (l1_cen,l1_angle) = self.detect_l1(image,'UR')
        elif jointPos1[0]>=0:
            (l1_cen,l1_angle) = self.detect_l1(image,'LR')
        elif jointPos1[1] >=0:
            (l1_cen,l1_angle) = self.detect_l1(image,'UL')
        else:
            (l1_cen,l1_angle) = self.detect_l1(image,'LL')
        l2_cen = np.zeros(2)
        l2_angle = 0
        #Determine which quadrant link 2 is pointing in and detect the angle and center
        if (jointPos2-jointPos1)[0]>=0 and (jointPos2-jointPos1)[1] >=0:
            (l2_cen,l2_angle) = self.detect_l2(image,'UR')
        elif (jointPos2-jointPos1)[0]>=0:
            (l2_cen,l2_angle) = self.detect_l2(image,'LR')
        elif (jointPos2-jointPos1)[1] >=0:
            (l2_cen,l2_angle) = self.detect_l2(image,'UL')
        else:
            (l2_cen,l2_angle) = self.detect_l2(image,'LL')
        l3_cen = np.zeros(2)
        l3_angle = 0
        #Determine which quadrant link 3 is pointing in and detect the angle and center
        if (jointPos3-jointPos2)[0]>=0 and (jointPos3-jointPos2)[1] >=0:
            (l3_cen,l3_angle) = self.detect_l3(image,'UR')
        elif (jointPos3-jointPos2)[0]>=0:
            (l3_cen,l3_angle) = self.detect_l3(image,'LR')
        elif (jointPos3-jointPos2)[1] >=0:
            (l3_cen,l3_angle) = self.detect_l3(image,'UL')
        else:
            (l3_cen,l3_angle) = self.detect_l3(image,'LL')

        #Combine all angles together as the joint angles and normalize to make sure they are between pi and -pi
        jointAngle1 = l1_angle

        jointAngle2 = l2_angle-l1_angle
        jointAngle2 = self.angle_normalize(jointAngle2)

        jointAngle3 = l3_angle-l2_angle
        jointAngle3 = self.angle_normalize(jointAngle3)
        return np.array([jointAngle1,jointAngle2,jointAngle3])

    def detect_ee(self,image):
        #Detect the end effector location
        return self.detect_blue(image)

    def rotation_matrix(self, angle):
        #Calculate the 2D rotation matrix
        return np.matrix([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])

    def link_transform(self,angle):
        #Calculate the Homogenoeous transformation matrix from rotation and translation
        rotation = np.matrix(np.eye(3,3))
        rotation[0:2,0:2] = self.rotation_matrix(angle)
        translation = np.matrix(np.eye(3,3))
        translation[0,2] = 1
        return rotation*translation

    def FK(self,joint_angles):
        #Forward Kinematics to calculate end effector location
        #Each link is 1m long
        #Calculate transformation matrix of each link
        j1_transform = self.link_transform(joint_angles[0])
        j2_transform = self.link_transform(joint_angles[1])
        j3_transform = self.link_transform(joint_angles[2])
        #Combine transformation matrices
        total_transform = j1_transform*j2_transform*j3_transform
        return total_transform

    def FK_analytic(self,joint_angles):
        #Forward Kinematics using the analytic equation
        #Each link is 1m long
        #Use trigonometry to solve FK so cos for X and sin for Y
        x = np.cos(joint_angles[0])*1 + np.cos(joint_angles[0]+joint_angles[1])*1 + np.cos(joint_angles[0]+joint_angles[1]+joint_angles[2])*1
        y = np.sin(joint_angles[0])*1 + np.sin(joint_angles[0]+joint_angles[1])*1 + np.sin(joint_angles[0]+joint_angles[1]+joint_angles[2])*1
        theta = sum(joint_angles)
        return np.array([x,y,theta])



    def Jacobian(self,joint_angles):
        #Forward Kinematics to calculate end effector location
        #Each link is 1m long
        #initialize matrix for jacobian
        jacobian = np.zeros((3,3))
        #calculate each individual jacobian transform
        j1_transform = self.link_transform(joint_angles[0])
        j2_transform = self.link_transform(joint_angles[1])
        j3_transform = self.link_transform(joint_angles[2])
        #combine the transforms for each
        total_transform = j1_transform*j2_transform*j3_transform
        #obtain end effector cartesian location
        ee_pos = total_transform[0:2,2]
        #obtain joint 3 cartesian location
        j3_pos = (j1_transform*j2_transform)[0:2,2]
        #obtain joint 2 cartesian location
        j2_pos = (j1_transform)[0:2,2]
        #obtain joint 1 cartesian location
        j1_pos = np.zeros((2,1))
        #Initialize vector containing axis of rotation, for planar robots rotating around z this is constant
        z_vector = np.array([0,0,1])
        #Calculate geometric jacobian using equations
        pos_3D = np.zeros(3)
        pos_3D[0:2] = (ee_pos-j1_pos).T
        jacobian[0:2,0] = np.cross(z_vector,pos_3D)[0:2]
        pos_3D[0:2] = (ee_pos-j2_pos).T
        jacobian[0:2,1] = np.cross(z_vector,pos_3D)[0:2]
        pos_3D[0:2] = (ee_pos-j3_pos).T
        jacobian[0:2,2] = np.cross(z_vector,pos_3D)[0:2]
        jacobian[2,:] = 1
        return jacobian

    def Jacobian_analytic(self,joint_angles):
        #Forward Kinematics using the analytic equation
        #Each link is 1m long
        #Use trigonometry from FK_analytic and differentiate it with respect to time.
        jacobian = np.zeros((3,3))
        jacobian[0,0] = -np.sin(joint_angles[0])*1 - np.sin(joint_angles[0]+joint_angles[1])*1 - np.sin(joint_angles[0]+joint_angles[1]+joint_angles[2])*1

        jacobian[0,1] = - np.sin(joint_angles[0]+joint_angles[1])*1 - np.sin(joint_angles[0]+joint_angles[1]+joint_angles[2])*1

        jacobian[0,2] = - np.sin(joint_angles[0]+joint_angles[1]+joint_angles[2])*1

        jacobian[1,0] = np.cos(joint_angles[0])*1 + np.cos(joint_angles[0]+joint_angles[1])*1 + np.cos(joint_angles[0]+joint_angles[1]+joint_angles[2])*1

        jacobian[1,1] = np.cos(joint_angles[0]+joint_angles[1])*1 + np.cos(joint_angles[0]+joint_angles[1]+joint_angles[2])*1

        jacobian[1,2] = np.cos(joint_angles[0]+joint_angles[1]+joint_angles[2])*1

        jacobian[2,0] = 1
        jacobian[2,1] = 1
        jacobian[2,2] = 1
        return jacobian

    def IK(self, current_joint_angles, desired_position):
        #Calculate current position and error in position in task space
        curr_pos = self.FK(current_joint_angles)[0:2,2]
        pos_error = desired_position - np.squeeze(np.array(curr_pos.T))
        #Calculate Jacobian
        Jac = np.matrix(self.Jacobian_analytic(current_joint_angles))[0:2,:]
        Jac_inv = Jac.T
        #If the Jacobian is low rank (<2) then use the transpose otherwise use psuedo-inverse
        if(np.linalg.matrix_rank(Jac,0.4)<2):
            Jac_inv = Jac.T
        else:
            Jac_inv = Jac.T*np.linalg.inv(Jac*Jac.T)
        #Apply inverted jacobian on the position error in task space to get qdot
        q_dot = Jac_inv*np.matrix(pos_error).T
        return np.squeeze(np.array(q_dot.T))

    def js_pd_control(self, current_joint_angles, current_joint_velocities, desired_joint_angles):
        P = np.array([1000,1000,1000])
        D = np.array([70,50,20])
        return np.diag(P)*np.matrix(self.angle_normalize(desired_joint_angles-current_joint_angles)).T-np.diag(D)*np.matrix(current_joint_velocities).T

    def ts_pd_control(self, current_joint_angles, current_joint_velocities, desired_position):
        #Calculate the torque required to reach the desired position using a PD controller (TASK SPACE)
        #Assume desired velocities are zero
        P = np.array([1000,1000])
        D = np.array([50,50])
        J = self.Jacobian(current_joint_angles)[0:2,:]
        #Obtain end effector velocity from equation x_dot = Jacobian * q_dot
        xd = J*np.matrix(current_joint_velocities).T
        curr_pos = self.FK(current_joint_angles)[0:2,2]

        return J.T*(np.diag(P)*(np.matrix(desired_position).T-curr_pos)-np.diag(D)*xd)

    def grav(self,current_joint_angles):
        #Calculate distance along x each joint is at
        h1 = math.cos(current_joint_angles[0])
        h12 = math.cos(current_joint_angles[0]+current_joint_angles[1])
        h123 = math.cos(current_joint_angles[0]+current_joint_angles[1]+current_joint_angles[2])
        #Calculate torque due to gravity at each link center of mass multiplied by the acceleration due to gravity
        #Important to note each one is with respect to the joint it is applied to so joint 1 has to compensate for the
        #mass of the first second and third link, whereas joint 2 is only the second and third and joint 3 only depends
        #on the third link
        return np.matrix([(h1*0.5+(h1+0.5*h12)+(h1+h12+0.5*h123))*9.81,((0.5*h12)+(h12+0.5*h123))*9.81,(0.5*h123)*9.81]).T

    def js_pd_grav_control(self, current_joint_angles, current_joint_velocities, desired_joint_angles):
        #Calculate the torque required to reach the desired joint angles using a PD controller with gravity compensation (JOINT SPACE)
        #Assume desired velocities are zero
        P = np.array([1000,1000,1000])
        D = np.array([70,50,20])
        grav_torques = self.grav(current_joint_angles)
        return np.diag(P)*np.matrix(self.angle_normalize(desired_joint_angles-current_joint_angles)).T-np.diag(D)*np.matrix(current_joint_velocities).T + grav_torques

    def ts_pd_grav_control(self, current_joint_angles, current_joint_velocities, desired_position):
        #Calculate the torque required to reach the desired position using a PD controller with gravity compensation (TASK SPACE)
        #Assume desired velocities are zero
        P = np.array([50,50])
        D = np.array([30,30])
        J = self.Jacobian(current_joint_angles)[0:2,:]
        xd = J*np.matrix(current_joint_velocities).T
        curr_pos = self.FK(current_joint_angles)[0:2,2]
        grav_torques = self.grav(current_joint_angles)
        #Apply gravity torques along with normal task space pd control
        return J.T*(np.diag(P)*(np.matrix(desired_position).T-curr_pos)-np.diag(D)*xd) + grav_torques

    def coordinate_convert(self,pixels):
        #Converts pixels into metres
        return np.array([(pixels[0]-self.env.viewerSize/2)/self.env.resolution,-(pixels[1]-self.env.viewerSize/2)/self.env.resolution])

    def go(self):
        #The robot has several simulated modes:
        #These modes are listed in the following format:
        #Identifier (control mode) : Description : Input structure into step function

        #POS : A joint space position control mode that allows you to set the desired joint angles and will position the robot to these angles : env.step((np.zeros(3),np.zeros(3), desired joint angles, np.zeros(3)))
        #POS-IMG : Same control as POS, however you must provide the current joint angles and velocities : env.step((estimated joint angles, estimated joint velocities, desired joint angles, np.zeros(3)))
        #VEL : A joint space velocity control, the inputs require the joint angle error and joint velocities : env.step((joint angle error (velocity), estimated joint velocities, np.zeros(3), np.zeros(3)))
        #TORQUE : Provides direct access to the torque control on the robot : env.step((np.zeros(3),np.zeros(3),np.zeros(3),desired joint torques))
        self.env.controlMode="TORQUE"
        #Run 100000 iterations
        prev_JAs = np.zeros(3)
        prev_jvs = collections.deque(np.zeros(3),1)
        #self.env.D_gains = np.array([40,40,40])
        #self.env.P_gains = np.array([400,400,400])
        self.env.D_gains[0] = 80
        self.env.enable_gravity(True)
        for _ in range(1000000):
            #The change in time between iterations can be found in the self.env.dt variable
            dt = self.env.dt
            #self.env.render returns and rgb_array containing the image of the robot
            rgb_array = self.env.render()
            detectedJointAngles = self.detect_joint_angles(rgb_array)
            prev_jvs.append(self.angle_normalize(detectedJointAngles-prev_JAs))
            #print (sum(prev_jvs)/len(prev_jvs))
            detectedJointVels = (sum(prev_jvs)/len(prev_jvs))/dt
            prev_JAs = detectedJointAngles
            jointAngles = np.array([3.14,-2,-1.14])
            #jointAngles = np.array([0,3.14,3.14])
            ee_target = self.detect_target(rgb_array)
            jointAngles = self.IK(detectedJointAngles,ee_target)
            #self.env.step((detectedJointAngles,detectedJointVels,jointAngles, np.zeros(3)))
            #self.env.step((jointAngles,detectedJointVels,np.zeros(3), np.zeros(3)))
            trqs = self.ts_pd_grav_control(detectedJointAngles,detectedJointVels,ee_target)
            #trqs = self.grav(detectedJointAngles)
            self.env.step((np.zeros(3),np.zeros(3),np.zeros(3),trqs))
            #The step method will send the control input to the robot, the parameters are as follows: (Current Joint Angles/Error, Current Joint Velocities, Desired Joint Angles, Torque input)

#main method
def main():
    reach = MainReacher()
    reach.go()

if __name__ == "__main__":
    main()
