import tensorflow as tf
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import chess
from scipy.spatial import distance as dist
import copy
import matplotlib.patches as patches
from sklearn.cluster import KMeans

import PIL.Image as Image
import base64
import io
from matplotlib import cm
import math

import Utils.ImageProcessingUtils as df

''' -------------------------------------- LOAD ML MODEL --------------------------------------'''

def loadModels():
  clf_save_path = "/home/trapoom555/fcb_ws/src/ai/src/Model/CNNClassification_150421_val_acc0p9393"
  clf = tf.keras.models.load_model(clf_save_path)
  return clf

''' -------------------------------------- LANDMARK DETECTION --------------------------------------'''

def order_points(pts):
    xSorted = pts[np.argsort(pts[:, 0]), :]

    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]
    return np.array([tl, tr, br, bl], dtype="float32"), (np.where(pts == tl)[0][0], np.where(pts == tr)[0][0], np.where(pts == br)[0][0], np.where(pts == bl)[0][0])

np.set_printoptions(suppress=True, linewidth=200)  # Better printing of arrays
plt.rcParams['image.cmap'] = 'jet'  # Default colormap is jet


def calc_img_band_score(img1, img2):
    # print("image band score")
    med_img1 = np.median(img1)
    std_img1 = np.std(img1)
    med_img2 = np.median(img2)
    std_img2 = np.std(img2)

    flag_win = 0

    if 255 - med_img1 > med_img1:
        # img1 is black
        color_err = med_img1 + (255 - med_img2)
    else:
        # img1 is white
        color_err = (255 - med_img1) + med_img2
        flag_win = 1

    return 20*color_err + (std_img1 + std_img2), flag_win



# return the image and the matrix

def mapLatticeInverse(allx, ally, M):
  resx = []
  resy = []
  for i in range(len(allx)):
    invertH = np.matmul(np.linalg.inv(M), np.array([allx[i],ally[i],1]))
    px = invertH[0] / invertH[2]
    py = invertH[1] / invertH[2]
    resx.append(px)
    resy.append(py)
  return resx, resy

def getMatrixFromImage(img):

    # img = Image.open(io.BytesIO(bytes(byteArray)))
    img = np.array(img)
    img_orig = Image.fromarray(img)
    img_width, img_height = img_orig.size

    # Resize
    # aspect_ratio = min(500.0/img_width, 500.0/img_height)
    # new_width, new_height = (
    #     (np.array(img_orig.size) * aspect_ratio)).astype(int)
    # img = img_orig.resize((new_width, new_height), resample=Image.BILINEAR)
    img = img_orig
    img_rgb = img
    img = img.convert('L')  # grayscale
    img = np.array(img)
    img_rgb = np.array(img_rgb)
    # cv2.imshow('test', img_rgb)
    # cv2.waitKey(0)
    M, ideal_grid, grid_next, grid_good, spts = df.findChessboard(img)

    # xy-unwarp -> the inner points of the inner chessboard
    # board_outline -> the corners (they are five because the first one is repeated)
    # boarder_points_?? -> the edges (?? edge of board: boarder_points_01 = edge from corner 0 to 1)

    # View
    if M is not None:
        # generate mapping for warping image
        M, _ = df.generateNewBestFit((ideal_grid+8)*32, grid_next, grid_good)
        img_warp = cv2.warpPerspective(
            img, M, (17*32, 17*32), flags=cv2.WARP_INVERSE_MAP)

        best_lines_x, best_lines_y = df.getBestLines(img_warp)
        xy_unwarp = df.getUnwarpedPoints(best_lines_x, best_lines_y, M)
        board_outline_unwarp = df.getBoardOutline(
            best_lines_x, best_lines_y, M)

        borders_points_01 = []
        borders_points_12 = []
        borders_points_23 = []
        borders_points_30 = []

        for i in range(0, len(xy_unwarp)):
            if i % 7 == 0:
                a, b = df.slope_intercept(
                    xy_unwarp[i, 0], xy_unwarp[i, 1], xy_unwarp[i+1, 0], xy_unwarp[i+1, 1])
                x_30, y_30 = df.line_intersection(([np.float32(0), b], [xy_unwarp[i, 0], xy_unwarp[i, 1]]), ([
                                                  board_outline_unwarp[3, 0], board_outline_unwarp[3, 1]], [board_outline_unwarp[0, 0], board_outline_unwarp[0, 1]]))
                x_12, y_12 = df.line_intersection(([-b/a, np.float32(0)], [xy_unwarp[i, 0], xy_unwarp[i, 1]]), ([
                                                  board_outline_unwarp[1, 0], board_outline_unwarp[1, 1]], [board_outline_unwarp[2, 0], board_outline_unwarp[2, 1]]))
                borders_points_30.append([x_30, y_30])
                borders_points_12.append([x_12, y_12])

            if i in range(0, 7):
                a, b = df.slope_intercept(
                    xy_unwarp[i, 0], xy_unwarp[i, 1], xy_unwarp[i+7, 0], xy_unwarp[i+7, 1])
                x_01, y_01 = df.line_intersection(([np.float32(0), b], [xy_unwarp[i, 0], xy_unwarp[i, 1]]), ([
                                                  board_outline_unwarp[0, 0], board_outline_unwarp[0, 1]], [board_outline_unwarp[1, 0], board_outline_unwarp[1, 1]]))
                x_23, y_23 = df.line_intersection(([-b/a, np.float32(0)], [xy_unwarp[i, 0], xy_unwarp[i, 1]]), ([
                                                  board_outline_unwarp[2, 0], board_outline_unwarp[2, 1]], [board_outline_unwarp[3, 0], board_outline_unwarp[3, 1]]))
                borders_points_01.append([x_01, y_01])
                borders_points_23.append([x_23, y_23])

        first_line = np.concatenate(([board_outline_unwarp[0]], borders_points_01, [
                                    board_outline_unwarp[1]]), axis=0)
        last_line = np.concatenate(([board_outline_unwarp[3]], borders_points_23, [
                                   board_outline_unwarp[2]]), axis=0)
        inner_lines = df.chunks(xy_unwarp, 7)
        for i in range(0, len(borders_points_12)):
            inner_lines[i] = np.concatenate(
                ([borders_points_30[i]], inner_lines[i], [borders_points_12[i]]), axis=0)

        matrix = np.vstack(([first_line], inner_lines, [last_line]))
        # We use the Matrix here
        innerCorner = np.array([matrix[0][0], matrix[0][-1], matrix[-1][0], matrix[-1][-1]])
        # for i in range(len(innerCorner)):
        #     ic = innerCorner[i]
        #     cv2.circle(img_rgb,ic.astype(np.int), 5, (0,255,0), -1)
        #     img_rgb = cv2.putText(img_rgb, str(i), ic.astype(np.int),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,255,0))

        pts1, sortOrder = order_points(innerCorner)
        pts2 = np.float32([[0,0],[400,0],[400,400],[0,400]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        outerCornerX_img_coord1,outerCornerY_img_coord1 = mapLatticeInverse([0,400,0,400], [-30,-30,430,430], M)
        outerCornerX_img_coord2, outerCornerY_img_coord2 = mapLatticeInverse([-30,-30,430,430], [0,400,0,400], M)
        for i in range(len(outerCornerX_img_coord1)):
            cv2.circle(img_rgb, (int(outerCornerX_img_coord1[i]), int(outerCornerY_img_coord1[i])), 5, (0,0,255), -1)
            cv2.circle(img_rgb, (int(outerCornerX_img_coord2[i]), int(outerCornerY_img_coord2[i])), 5, (255,0,0), -1)
            img_rgb = cv2.putText(img_rgb, str(i), (int(outerCornerX_img_coord1[i]), int(outerCornerY_img_coord1[i])),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,0,255))
            img_rgb = cv2.putText(img_rgb, str(i), (int(outerCornerX_img_coord2[i]), int(outerCornerY_img_coord2[i])),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(255,0,0))
        cv2.imshow('ha', img_rgb)
        cv2.waitKey(0)

        outerCorner1 = np.float32(np.concatenate((np.array(outerCornerX_img_coord1).reshape(-1,1),np.array(outerCornerY_img_coord1).reshape(-1,1)),axis = -1))
        outerCorner1, sortOrder = order_points(outerCorner1)
        pts2_expand = np.float32([[0,0],[430,0],[430,430],[0,430]])
        M_guess_1 = cv2.getPerspectiveTransform(outerCorner1,pts2_expand)
        dst1 = cv2.warpPerspective(img,M_guess_1,(430,430))
        # cv2.imshow('ha_warp', dst1)
        # cv2.waitKey(0)
        upperImage = dst1[:30,:]
        # cv2.imshow('ha_warp_up', upperImage)
        # cv2.waitKey(0)
        lowerImage = dst1[400:,:]
        # cv2.imshow('ha_warp_down', lowerImage)
        # cv2.waitKey(0)
        err_img1, flag_win1 = calc_img_band_score(upperImage, lowerImage)


        outerCorner2 = np.float32(np.concatenate((np.array(outerCornerX_img_coord2).reshape(-1,1),np.array(outerCornerY_img_coord2).reshape(-1,1)),axis = -1))
        outerCorner2, sortOrder = order_points(outerCorner2)
        M_guess_2 = cv2.getPerspectiveTransform(outerCorner2,pts2_expand)
        dst2 = cv2.warpPerspective(img,M_guess_2,(430,430))
        # cv2.imshow('ha_warp2', dst2)
        # cv2.waitKey(0)
        leftImage = dst2[:,:30]
        # cv2.imshow('ha_warp2_up', leftImage)
        # cv2.waitKey(0)
        rightImage = dst2[:,400:]
        # cv2.imshow('ha_warp2_down', rightImage)
        # cv2.waitKey(0)
        err_img2, flag_win2 = calc_img_band_score(leftImage, rightImage)

        if err_img1 > err_img2:
            # print("blue wins")
            if flag_win2: # left image is white
                #pass
                ans = np.array([outerCorner2[0], outerCorner2[3], outerCorner2[2], outerCorner2[1]])
                # cv2.imshow('white', leftImage)
                # cv2.imshow('black', rightImage)
            else: # right image is white
                # pass
                ans = np.array([outerCorner2[2], outerCorner2[1], outerCorner2[0], outerCorner2[3]])
            #     cv2.imshow('white', rightImage)
            #     cv2.imshow('black', leftImage)
            # cv2.imshow('ha_warp', dst2)
        else:
            # print("red wins")
            if flag_win1: # upper image is white
                # pass
                ans = np.array([outerCorner1[1], outerCorner1[0], outerCorner1[3], outerCorner1[2]])
                # cv2.imshow('black', lowerImage)
                # cv2.imshow('white', upperImage)
            else:
                #pass
                ans = np.array([outerCorner1[3], outerCorner1[2], outerCorner1[1], outerCorner1[0]])
            #     cv2.imshow('white', lowerImage)
            #     cv2.imshow('black', upperImage)
            # cv2.imshow('ha_warp', dst1)
        print("err red : {}", err_img1)
        print("err blue : {}", err_img2)

        img_show = np.dstack([img, img, img])
        for i in range(len(ans)):
            a = ans[i]
            cv2.circle(img_show,a.astype(np.int), 5, (0,255,0), -1)
            img_show = cv2.putText(img_show, str(i), a.astype(np.int),fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0,200,0))
        ans = ans.flatten()
        cv2.imshow('ha', img_show)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return ans, img_show

    else:
        # cv2.imshow("Image", img)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        return None, None, None

''' -------------------------------------- CLASSIFICATION --------------------------------------'''

deCategorical = {
    0 : 'none',
    1 : 'k',
    2 : 'q',
    3 : 'r',
    4 : 'b',
    5 : 'n',
    6 : 'p'
}


def order_points(pts):
  xSorted = pts[np.argsort(pts[:, 0]), :]

  leftMost = xSorted[:2, :]
  rightMost = xSorted[2:, :]

  leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
  (tl, bl) = leftMost

  D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
  (br, tr) = rightMost[np.argsort(D)[::-1], :]
  return np.array([tl, tr, br, bl], dtype="float32"), (np.where(pts == tl)[0][0], np.where(pts == tr)[0][0], np.where(pts == br)[0][0], np.where(pts == bl)[0][0])

def notationToSquareIdx(notation):
  charNot = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
  numNot =  ['1', '2', '3', '4', '5', '6', '7', '8']
  n = charNot.index(notation[0])
  m = numNot.index(notation[1])
  return 8*m + n

def findLattice(sortOrder):
  latx = np.linspace(50,350,9)
  laty = (np.linspace(50,350,10))
  laty = (laty + (laty[-1] - laty[-2])/2)[:-1]
  if((sortOrder[1]==2 and sortOrder[2] == 1) or (sortOrder[0] == 1 and sortOrder[3] == 2)):
    tmp = np.transpose([np.tile(latx, len(laty)), np.repeat(laty, len(latx))])
  else:
    tmp = np.transpose([np.tile(laty, len(latx)), np.repeat(latx, len(laty))])
  allx = tmp[:,0]
  ally = tmp[:,1]
  return allx, ally

def mapLatticeInverse(allx, ally, M):
  resx = []
  resy = []
  for i in range(len(allx)):
    invertH = np.matmul(np.linalg.inv(M), np.array([allx[i],ally[i],1]))
    px = invertH[0] / invertH[2]
    py = invertH[1] / invertH[2]
    resx.append(px)
    resy.append(py)
  return resx, resy

def Classification(im, points, model):
  colorupper = 45
  colorlower = 53
  #im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
  board = chess.Board('8/8/8/8/8/8/8/8 w - - 0 1')
  # points 8 x 1
  global deOnehot
  xs = [points[0], points[2], points[4], points[6]]
  ys = [points[1], points[3], points[5], points[7]]
  pts1 = np.float32(np.concatenate((np.array(xs).reshape(-1,1),np.array(ys).reshape(-1,1)),axis = -1))
  pts1, sortOrder = order_points(pts1)
  pts2 = np.float32([[0,0],[300,0],[300,300],[0,300]])
  M = cv2.getPerspectiveTransform(pts1,pts2)
  # do some Padding
  padcornerx, padcornery = mapLatticeInverse([-50,350,350,-50], [-50,-50,350,350], M)
  pts3 = np.float32(np.concatenate((np.array(padcornerx).reshape(-1,1),np.array(padcornery).reshape(-1,1)),axis = -1))
  pts4 = np.float32([[0,0],[400,0],[400,400],[0,400]])
  padM = cv2.getPerspectiveTransform(pts3,pts4)
  dst = cv2.warpPerspective(im,padM,(400,400))
  allx, ally = findLattice(sortOrder)
  halfX = int(np.abs(allx[1] - allx[0]) / 2)
  halfY = int(np.abs(ally[10] - ally[0]) / 2)
  charNot = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
  numNot =  ['1', '2', '3', '4', '5', '6', '7', '8']
  Xglob, Yglob = mapLatticeInverse(allx, ally, padM)
  counter = 10
  colors = []
  predicts = []
  for i in range(8):
    for j in range(8):
      if(j == 0 and counter != 10):
        counter += 1
      if(sortOrder[0] == 0):
        notation = charNot[i] + numNot[j]
      elif(sortOrder[0] == 1):
        notation = charNot[7-j] + numNot[i]
      elif(sortOrder[0] == 2):
        notation = charNot[7-i] + numNot[7-j]
      else:
        notation = charNot[j] + numNot[7-i]
      cropimg = dst[max(int(ally[counter] - 3.5*halfY),0):min(int(ally[counter] + 1.5*halfY), 400), max(int(allx[counter] - 3.5*halfX),0):min(int(allx[counter] + 1.5*halfX), 400)]
      resized_color = cv2.resize(cropimg,(100,100))
      resized_color_rec = copy.deepcopy(resized_color)
      cv2.rectangle(resized_color_rec, (colorupper, colorupper), (colorlower, colorlower), (255,0,0), 1)
      plt.imshow(resized_color_rec[:,:,::-1])
      plt.axis('off')
      # plt.show()
      # average 5x5 sampling colors
      colors.append(resized_color[colorupper:colorlower,colorupper:colorlower,:].reshape(-1,3).mean(axis = 0))
      resized = cv2.cvtColor(resized_color, cv2.COLOR_BGR2GRAY)
      # 48-52
      # plt.imshow(resized, cmap = 'gray')
      # plt.axis('off')
      # plt.show()
      predict = model.predict(resized.reshape(-1,100,100,1))
      predicts.append(predict)
      # print(deCategorical[np.argmax(predict[0])])
      counter += 1
  # Color Cluster
  kmeans = KMeans(n_clusters=2, random_state=0).fit(colors)
  print(len(colors))
  for i in range(8):
    for j in range(8):
      if(sortOrder[0] == 0):
        notation = charNot[i] + numNot[j]
      elif(sortOrder[0] == 1):
        notation = charNot[7-j] + numNot[i]
      elif(sortOrder[0] == 2):
        notation = charNot[7-i] + numNot[7-j]
      else:
        notation = charNot[j] + numNot[7-i]
      predict = predicts[8*i+j]
      if(deCategorical[np.argmax(predict[0])] != 'none'):
        sym = deCategorical[np.argmax(predict[0])]
        color = kmeans.predict(colors[8*i+j].reshape(1,-1))
        if color[0] == 1:
          sym = sym.upper()
        board.set_piece_at(notationToSquareIdx(notation), chess.Piece.from_symbol(sym))
      counter += 1
  return board, dst

''' -------------------------------------- COMBINED --------------------------------------'''


def completePipeline(img):
  clf = loadModels()
  points, img_show = getMatrixFromImage(img)
  print("Complete Detect All Points")
  print("Points : {}".format(points))
  b, dst = Classification(img, points, clf)
  return b, dst
