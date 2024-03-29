#! /usr/bin/env python

import cv2
import rospy
import numpy as np


class Cards:
    def __init__(self, img = []):
        self.img = img

    def get_features(self, kernel_shape, min_area = 2000, max_area = 120000):
        # Preprocessing image to get features.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_shape)
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray,11,17,17) # Filtering noise
        blur = cv2.GaussianBlur(gray, kernel_shape, 0)
        thresh = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2, 2)
        erode = cv2.erode(thresh, kernel, iterations = 1)
        canny = cv2.Canny(blur, blur.mean()*0.01, blur.mean()*1.5)
        img, contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Initialising return lists
        cards_contours = []
        cards_corners = []
        cards_hierarchy = []

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            perimeter = 0.1*cv2.arcLength(contours[i],True)
            corner = cv2.approxPolyDP(contours[i],perimeter,True)

            # Contour and corners will only be appended to the list, only
            # if the area is within range (max_area, min_area), contains 4
            # corners and have no parents (hierarchy).
            if (area > min_area) and (area < max_area) and (len(corner) == 4) and (hierarchy[0, i, 3] == -1):
                cards_contours.append(contours[i])
                cards_corners.append(corner)
                cards_hierarchy.append(hierarchy[0])

        return cards_corners, cards_contours, cards_hierarchy

    def extract_card(self, corner):
        # Determining the size of height and width of the card by adding its x and y positions
        a_dis = abs(corner[0, 0, 0] - corner[1, 0, 0]) + abs(corner[0 , 0, 1] - corner[1, 0, 1])
        b_dis = abs(corner[1, 0, 0] - corner[2, 0, 0]) + abs(corner[1 , 0, 1] - corner[2, 0, 1])
        # Flipping corners to fix contours having an external and internal side, causing the warp to flip.
        if a_dis > b_dis:
            pts1 = np.float32([corner[0, 0], corner[3, 0], corner[1, 0], corner[2, 0]])
        else:
            pts1 = np.float32([corner[1, 0], corner[0, 0], corner[2, 0], corner[3, 0]])
        pts2 = np.float32([[0,0],[200, 0], [0, 300], [200, 300]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(self.img, matrix, (200, 300))

    def extract_suit(self, corners):
        suits = []
        if len(corners) is not 0:
            for corner in corners:
                card = self.extractCard(corner)
                # Preprocessing extracted cards.
                gray = cv2.cvtColor(card, cv2.COLOR_BGR2GRAY)
                gray = cv2.bilateralFilter(gray,11,17,17)
                thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2, 2)
                # Appending cropped image of suit and rank to suits array.
                suits.append(thresh[7:77, 5:35])
        return suits
