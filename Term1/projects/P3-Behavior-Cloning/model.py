# # Self-Driving Car Engineer Nanodegree
# 
# ## Deep Learning
# 
# ## Project: Behavioral Cloning
# 
# 

# ## Step 0: Import packages
import numpy as np
import pandas as pd
import cv2
from sklearn.model_selection import train_test_split

import os
import sys

from keras.models import load_model
from keras.models import Sequential
from keras.layers import Dense, Activation, Dropout,Convolution2D,MaxPooling2D,Flatten,Lambda
from keras.optimizers import Adam
from keras.models import model_from_json
import json

import matplotlib
import matplotlib.pyplot as plt


#Define constants
RESIZE_ROWS = 64
RESIZE_COLS = 64


# ## Step 1: Load The Data
data_dir = './'
drive_csv = 'driving_log.csv'

train_data = pd.read_csv(data_dir+drive_csv,names=['center', 'left','right','steering','throttle','brake','speed'])
train_data.head(5)

#Split the data into training and validation
x_train_data = train_data[['center','left','right']]
y_train_data = train_data['steering']

#In this step, we split the training samples into training set and validation set. Validation set is a test set
# that shows how good our model is learning during the training phase.

print('In validation set creation')
X_train_1, X_val_1, y_train_1, y_val_1 = train_test_split(
   x_train_data,
   y_train_data,
   test_size=0.2,
   random_state=7322869
)
print('In validation set creation - complete')

# Convert the dataframe into numpy array
X_left   = X_train_1['left'].values
X_right  = X_train_1['right'].values
X_center = X_train_1['center'].values
y_train_2  = y_train_1.values
X_val_center = X_val_1['center'].values
y_val_2 =  y_val_1.values


y_train = y_train_2.astype(np.float32)
y_val   = y_val_2.astype(np.float32)

print(X_left[2])
print('Completed the loading of csv file')


#Histogram of the steering angles.
#I plot the steering angles to see the distribution.
#plt.hist(train_data.steering)

#Read an image randomly from the csv.
# I assume the side camera to be around 1.0 meters off the center and the offset to the left or right 
# should be be corrected over the 20.0 meters
# using s = r*theta
offset=1.0 
dist=20.0

DELTA = offset/dist * 360/( 2*np.pi) / 25.0

def read_random_image(X_train,X_left,X_right,Y_train):
     m = np.random.randint(0,len(Y_train))
     lcr = np.random.randint(0,3)

     steer = Y_train[m]
     if lcr == 0:
         image = plt.imread(X_left[m].replace(' ',''))
         steer += DELTA
     elif lcr == 1:
         image = plt.imread(X_train[m].replace(' ',''))
     elif lcr == 2:
         image = plt.imread(X_right[m].replace(' ',''))
         steer -= DELTA
     else:
         print ('Invalid lcr value :',lcr )
    
     return image,steer    


#IMAGE AUGMENTATION - Affine transformation - Shear
#I apply the affine transformation of shear to synthesize curved paths from straight paths
SCALE_FACTOR = 100.0

def shear_image(img, steering, shear_angle_range):
    rows,cols,ch = img.shape
    dTheta = np.random.randint(-shear_angle_range, shear_angle_range) #Theta in radians.
    dThetaRadians = dTheta * (np.pi/180.0)
    dX = (dThetaRadians) * (rows/2)
    offset_point = [cols/2+dX,rows/2]
    
    srcPts = np.float32([[0,rows],[cols,rows],[cols/2,rows/2]])
    dstPts = np.float32([[0,rows],[cols,rows],offset_point])
    
    dsteering = dTheta / SCALE_FACTOR    
    M = cv2.getAffineTransform(srcPts, dstPts)
    img_sheared = cv2.warpAffine(img,M,(cols,rows),borderMode=1)
    new_steering = steering + dsteering    
    
    return img_sheared, new_steering


#IMAGE AUGMENTATION - Affine transformation - Translation
#I apply translation operation on the image to move away from the center of the lane. 
#Corresponding to this translation I add an offset to the steering angle.
ANGLE_OFFSET_PER_PIXEL = 0.01

def translate_image(img, steering, translate_range=100):
    rows,cols,ch = img.shape

    tr_x = translate_range*np.random.uniform()-translate_range/2
    M = np.float32([[1,0,tr_x],[0,1,0]])
    img_translated = cv2.warpAffine(img,M,(cols,rows))
    new_steering = steering + ((-tr_x/(translate_range/2)) * ANGLE_OFFSET_PER_PIXEL)  #Notice the negative sign in front of tr_x
    
    return img_translated, new_steering



#IMAGE AUGMENTATION - brightness manipulation
#For changing the brightness, we modify from RGB to HSV and then change the 'V' value.
BRIGHTNESS_OFFSET = 0.3
def alter_brightness_image(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #convert it to hsv
    scale_factor = BRIGHTNESS_OFFSET + np.random.uniform()
    hsv[:,:,2] = scale_factor*hsv[:,:,2]
    new_image = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    
    return new_image


#IMAGE AUGMENTATION - Flip the images.
#As observed in the histogram, the steering angles are not evenly distributed about the 0 value.
#There are more images with negative steering angles(taking left).
def flip_image(image,steering):
    need_flip=np.random.randint(0,2)
    if need_flip==0:
        image=cv2.flip(image,1)
        steering = -steering
        
    return image,steering


#IMAGE AUGMENTATION : Crop image and resize
#We derive motivation from https://chatbotslife.com/using-augmentation-to-mimic-human-driving-496b569760a9#.481wwl37b
# to crop the image, so as to reduce the unwanted portion, namely the bonnet of the car and the horizon, in steering the car.

#Finally we resize the image to 64x64 size and then feed it to the network.

def crop_resize_image(img):
   rows,cols,ch = img.shape
   HORIZON = 60
   BONNET = 136
   img_crop = img[HORIZON:BONNET, 0:cols, :]
   img_resize = cv2.resize(img_crop,(RESIZE_COLS,RESIZE_ROWS), interpolation=cv2.INTER_AREA)    
   return img_resize


#Augmentation pipeline
#I read a random image from the list and pass it through the 
#shear, crop, brighness_alter, crop-resize pipeline to generate a new image.

def generate_augmented_image(X_center,X_left,X_right,y_train):
     img, steer = read_random_image(X_center,X_left,X_right,y_train)
    
     img,steer = shear_image(img, steer, shear_angle_range=10)   
    
     img, steer = translate_image(img, steer, translate_range=80)

     img, steer =  flip_image(img, steer)
    
     img =  crop_resize_image(img)
    
     img = alter_brightness_image(img)
    
     return img,steer



#Generate validation set by passing it through the augmentation pipeline.
def generate_validation_set(X_val,y_val):
    X = np.zeros((len(X_val),RESIZE_ROWS, RESIZE_COLS, 3))
    Y = np.zeros(len(X_val))
    for i in range(len(X_val)):
        img  = plt.imread(X_val[i])
        Y[i] = y_val[i]
        X[i] = crop_resize_image(img)
    return X,Y


#Generate training set
def generate_trainset_batch(X_train,X_left,X_right,Y_train,batch_size = 32):
    
    batch_images = np.zeros((batch_size, RESIZE_ROWS, RESIZE_COLS, 3))
    batch_steering = np.zeros(batch_size)
    while 1:
        for i_batch in range(batch_size):
            img, steer = generate_augmented_image(X_center,X_left,X_right,y_train)
            batch_images[i_batch] = img
            batch_steering[i_batch] = steer
        yield batch_images, batch_steering



batch_size=200
train_generator = generate_trainset_batch(X_center,X_left,X_right,y_train,batch_size)
X_val,Y_val = generate_validation_set(X_val_center,y_val)

print('X_train data type :',X_center.dtype)
print('Y_train data type :',y_train.dtype)
print('X_val data type :',X_val.dtype)
print('Y_val data type :',y_val.dtype)


#DEFINE THE KERAS MODEL
model = Sequential()
model.add(Lambda(lambda x: x/127.5 - 1.0,input_shape=(RESIZE_ROWS, RESIZE_COLS, 3)))
model.add(Convolution2D(24,5,5,border_mode='valid', activation='relu', subsample=(2,2)))
model.add(Convolution2D(36,5,5,border_mode='valid', activation='relu', subsample=(2,2)))
model.add(Convolution2D(48,5,5,border_mode='valid', activation='relu', subsample=(2,2)))
model.add(Convolution2D(64,3,3,border_mode='valid', activation='relu', subsample=(1,1)))
model.add(Convolution2D(64,3,3,border_mode='valid', activation='relu', subsample=(1,1)))
model.add(Flatten())
model.add(Dropout(0.5)) #New
model.add(Dense(1164, activation='relu'))
model.add(Dense(100, activation='relu'))
model.add(Dropout(0.5)) #New
model.add(Dense(50, activation='relu'))
model.add(Dense(10, activation='relu'))
model.add(Dense(1, activation='tanh'))
model.summary()


#Define training parameters.
model_json = 'model.json'
model_weights = 'model.h5'

adam = Adam(lr=1e-4, beta_1=0.9, beta_2=0.999, epsilon=1e-08, decay=0.0)

restart=True
if os.path.isfile(model_json) and restart:
    try:
        with open(model_json) as jfile:
            model = model_from_json(json.load(jfile))
            model.load_weights(model_weights)    
        print('loading trained model ...')
    except Exception as e:
        print('Unable to load model', model_name, ':', e)
        raise    

model.compile(optimizer=adam, loss='mse')


nb_epoch=10
history = model.fit_generator(train_generator,
                    samples_per_epoch=20000, nb_epoch=nb_epoch,
                    validation_data=(X_val,Y_val),verbose=1)

json_string = model.to_json()

print('Save the model')

try:
    os.remove(model_json)
    os.remove(model_weights)
except OSError:
    pass   

with open(model_json, 'w') as outfile:
    json.dump(json_string, outfile)
model.save_weights(model_weights)

print('Done')
