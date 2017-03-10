#!/usr/bin/env python

from __future__ import print_function
#importing some useful packages
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
# from sklearn import linear_model, datasets
import numpy as np
import cv2
# Import everything needed to edit/save/watch video clips
# from moviepy.editor import VideoFileClip
# from IPython.display import HTML
import os
import sys
# from sklearn.cluster import KMeans
# from sklearn import svm
# from sklearn.datasets import make_blobs
from PIL import Image
import math

import roslib
roslib.load_manifest('traxers_vision')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError




def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.

    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def find_lines_center_of_mass(lines):
    """
    Finds the center of mass ofthe convex hull of a list of x,y tuples

    """
    contour_points = []
    for line in lines:
        for x1,y1,x2,y2 in line:
            contour_points.append(np.array([np.array([x1,y1])]))
            contour_points.append(np.array([np.array([x2,y2])]))
        # find proper center of area
    contour_points = np.array(contour_points)
    hull = cv2.convexHull(contour_points)
    M = cv2.moments(hull)
    center_x = int(M['m10']/M['m00'])
    center_y = int(M['m01']/M['m00'])
    return center_x, center_y

def find_x_moment(line, x, length_thresh):
    """
    Finds the moment of area of a two point line (p1,p2), with respect to
    the vertical line at x. the points i.e. p1 and p2 are of the form
    [x1,y1]

    Lines of length less than length thresh will return nan
    """
    for x1,y1,x2,y2 in line:
        h = abs(y2-y1)
        #if the line is too short, ignore its contribution
        if h < length_thresh:
            return np.nan
        #else we have a rhombus
        b1 = 0
        b2 = 0
        if abs(x1 - x) > abs(x2 - x):
            b1 = x1 - x
            b2 = x2 - x
        else:
            b1 = x2 - x
            b2 = x1 - x
        moment = (b2 + 0.5 *(b1-b2))*h
    return moment

# def post_process_lines(lines, img_shape):
#     """
#     runs RANSAC over a list of points and returns the endpoints of
#     the linear line result
#
#     """
#
#     model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
#     X = []
#     y = []
#     # run ransac on segment points to fit better line
#     maxy = 0
#     miny = sys.maxsize
#     for line in lines:
#         for x1,y1,x2,y2 in line:
#             maxy = max(y1,max(maxy,y2))
#             miny = min(y1,min(miny,y2))
#             X.append(x1)
#             X.append(x2)
#             y.append(y1)
#             y.append(y2)
#     X = np.array(X).reshape(len(X),1)
#     y = np.array(y).reshape(len(y),1)
#
#     if len(X) > 1:
#         model_ransac.fit(y,X)
#         return model_ransac.predict(img_shape[0]), img_shape[0], model_ransac.predict(int(img_shape[0]*0.63)), int(img_shape[0]*0.63)
#
#     return np.nan, np.nan, np.nan, np.nan

# def split_left_right_lines(lines, center_x ,slopes):
#     """
#     Splits a list of lines into lines of the left of center_x
#     and lines on the right of center_x.
#
#     Assumes that the lines on a given side should have similar slopes
#     and thus also uses the slope information
#
#     """
#     left_lines = []
#     right_lines = []
#
#     slopes = np.array(slopes).reshape(len(slopes),1)
#     clusterer = KMeans(n_clusters=2, random_state=170,init=np.array([-20,20]).reshape(2,1))
#     clusterer.fit(slopes)
#     left_slope = min(clusterer.cluster_centers_)
#     right_slope = max(clusterer.cluster_centers_)
#     left_index = clusterer.predict(left_slope)
#     right_index = clusterer.predict(right_slope)
#     abs_slope_similarity = 10
#     #print(left_slope,right_slope)
#
#     for line in lines:
#         for x1, y1, x2, y2 in line:
#             moment = find_x_moment(line, center_x, 0)
#             slope = (y2-y1)/(x2-x1+0.05)
#             if np.isnan(moment) or np.isnan(slope) or np.isinf(slope):
#                 break
#             if moment < 0 and clusterer.predict(slope) == left_index and abs(slope - left_slope) < abs_slope_similarity:
#                 if x1 > center_x and x2 > center_x:
#                     print("wrong assignment to left ",x1,y1,x2,y2 )
#                 left_lines.append(line)
#             elif moment > 0 and clusterer.predict(slope) == right_index and abs(slope - right_slope) < abs_slope_similarity:
#                 if x1 < center_x and x2 < center_x:
#                     print("wrong assignment to left ",x1,y1,x2,y2 )
#                 right_lines.append(line)
#     return left_lines, right_lines

def draw_lines(img, lines, color=[255, 0, 0], thickness=10):
    """
    NOTE: this is the function you might want to use as a starting point once you want to
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).

    Think about things like separating line segments by their
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of
    the lines and extrapolate to the top and bottom of the lane.

    This function draws `lines` with `color` and `thickness`.
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    nx = img.shape[1]
    ny = img.shape[0]
    slopes = []
    for line in lines:
        for x1,y1,x2,y2 in line:
            line_length = (y2-y1)*(y2-y1) + (x2-x1) * (x2-x1)
            slope = ((y2-y1)/(x2-x1))
            if not np.isnan(slope) and not np.isinf(slope):
                slopes.append(slope)
            cv2.line(img,(x1,y1),(x2,y2),[0,0,255],thickness)

    #find center line
    #cv2.line(img,(center_x,0),(center_x,img.shape[0]),[0,0,255],thickness)

    #calculate determine left or right lines
    # left_lines, right_lines = split_left_right_lines(lines, center_x , slopes )

    # try out simple lines


    # x1,y1,x2,y2 = post_process_lines(left_lines,img.shape)
    # if not np.isnan(x1):
    #     cv2.line(img,(x1,y1), (x2, y2),[0,255,0],thickness )
    #
    # x1,y1,x2,y2 = post_process_lines(right_lines,img.shape)
    # if not np.isnan(x1):
    #     cv2.line(img,(x1,y1), (x2, y2),[255,0,0],thickness )




def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.

    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    slopes = draw_lines(line_img, lines)
    return line_img

# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, alpha=0.8, beta=1., gamma=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.

    `initial_img` should be the image before any processing.

    The result image is computed as follows:

    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, alpha, img, beta, gamma)

def process_image(image):

    #change to hsv space
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    # define range of yellow color in HSV
    lower_yellow = np.array([20,50,50])
    upper_yellow = np.array([30,255,255])


    # Threshold the HSV image to get only yellow colors and white
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_gray   = cv2.inRange(grayscale(image),210,255)

    mask = cv2.bitwise_or(mask_gray,mask_yellow)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(image,image, mask= mask)

    # set remaining pixels to max value
    gray = grayscale(res)
    gray[gray != 0] = 255

    # Define a kernel size and apply Gaussian smoothing
    kernel_size = 3
    gray = gaussian_blur(gray,kernel_size)

    # Focus on the
    imshape = image.shape
    nx = imshape[1]
    ny = imshape[0]
    vertices = np.array([[(0.15*nx,ny),(0.4*nx, 0.65*ny), (0.6*nx, 0.65*ny), (0.9*nx,ny)]], dtype=np.int32)
    gray = region_of_interest(gray,vertices)

    low_threshold = 150
    high_threshold = 250
    edges = canny(gray, low_threshold, high_threshold)

    rho = 5 # distance resolution in pixels of the Hough grid
    theta = 5*np.pi/180 # angular resolution in radians of the Hough grid
    threshold = 50     # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 10 #minimum number of pixels making up a line
    max_line_gap = 2 #maximum gap in pixels between connectable line segments
    line_image = hough_lines(gray, rho, theta, threshold, min_line_length, max_line_gap)

    color_gray = np.dstack((edges,edges,edges))
    lines_edges = cv2.addWeighted(image, 0.8, line_image, 1, 0)

    return lines_edges
    #return res
    return np.concatenate((image,np.concatenate((lines_edges,res), axis=1)), axis=1)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    lines_edges = process_image(cv_image)

    cv2.imshow("Image window", lines_edges)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(lines_edges, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
