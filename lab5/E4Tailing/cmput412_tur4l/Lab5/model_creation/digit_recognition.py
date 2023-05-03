import tensorflow as tf
import os
import cv2
import numpy as np
from matplotlib import pyplot as plt
from scipy.ndimage import rotate, shift
import random
import rosbag
from digit_bag_util import read_dict


def plt_showim(im):
    plt.imshow(im)
    plt.show()
    plt.cla()


def augment_dataset(x_train, y_train):
    aug_train_x = []
    aug_train_y = []
    nim = x_train.shape[0]
    DEGREES_RANGE = 8.
    SHIFT_RANGE = 2.
    for i in range(20):
        cur_x = np.zeros(x_train.shape, dtype=np.uint8)
        for j in range(nim):
            # plt_showim(self.x_train[j])
            rot = DEGREES_RANGE * (random.random() * 2 - 1.)
            shiftx = SHIFT_RANGE * (random.random() * 2 - 1.)
            shifty = SHIFT_RANGE * (random.random() * 2 - 1.)
            cur_x[j] = rotate(x_train[j], rot, reshape=False)
            cur_x[j] = shift(cur_x[j], (shiftx, shifty))
            # plt_showim(cur_x[j])
        aug_train_x.append(cur_x)
        aug_train_y.append(y_train)
    aug_train_x = np.concatenate(aug_train_x, axis=0)
    aug_train_y = np.concatenate(aug_train_y, axis=0)
    return aug_train_x, aug_train_y


class Recognizer:
    def __init__(self):
        mnist = tf.keras.datasets.mnist
        (self.x_train, self.y_train), (self.x_test, self.y_test) = mnist.load_data()

        for i in range(self.x_train.shape[0]):
            self.x_train[i] = cv2.dilate(self.x_train[i], np.ones((3, 3), np.uint8), iterations=1)
            # plt_showim(self.x_train[i])

        # with open('../dataset_model/TMNIST_Data.csv', 'r') as tminst:
        #     lines = tminst.readlines()
        #     x_train = np.zeros((len(lines), 28, 28), dtype=np.uint8)
        #     y_train = np.zeros(len(lines))
        #     for l in range(1, len(lines)):
        #         line = lines[l]
        #         words = line.split(',')[-(28 * 28 + 1):]  # some font name will contain ',' character
        #         label = int(words[0])
        #         for i in range(28):
        #             for j in range(28):
        #                 idx = i * 28 + j + 1
        #                 x_train[l][i][j] = int(words[idx])
        #         y_train[l] = label
                # plt_showim(self.x_train[l])
            # self.x_train = np.concatenate((self.x_train, x_train), axis=0)
            # self.y_train = np.concatenate((self.y_train, y_train), axis=0)
            # self.x_train_2 = x_train
            # self.y_train_2 = y_train

        bag = rosbag.Bag('collected_digit.bag')
        label_table = read_dict()
        x_train = []
        y_train = []
        for topic, msg, t in bag.read_messages(topics=[]):
            compressed_image = np.frombuffer(msg.data, np.uint8)
            seq = msg.header.seq
            if seq not in label_table:
                continue
            label = label_table[seq]
            im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)[:, :, 0]
            x_train.append(im)
            y_train.append(label)
        x_train, y_train = augment_dataset(np.array(x_train, dtype=np.uint8), np.array(y_train))
        # self.x_train = np.concatenate((self.x_train, x_train), axis=0)
        # self.y_train = np.concatenate((self.y_train, y_train), axis=0)
        self.x_train_3 = x_train
        self.y_train_3 = y_train

        self.model = None

    def train_data(self):
        print("TRAINING THE DATA")
        DROPOUT_RATE = .1
        base_model = tf.keras.applications.Xception(
        weights='imagenet',  # Load weights pre-trained on ImageNet.
        input_shape=(28, 28),
        include_top=False)  # Do not include the ImageNet classifier at the top.
        base_model.trainable = False
        inputs = tf.keras.Input(shape=(28, 28))
        x = base_model(inputs, training=False)
        x = tf.keras.layers.Flatten(input_shape=(28, 28))(x)
        x = tf.keras.layers.Dense(250, activation="relu")(x)
        x = tf.keras.layers.Dropout(DROPOUT_RATE)(x)
        x = tf.keras.layers.Dense(100, activation="relu")(x)
        outputs = tf.keras.layers.Dense(10, activation="softmax")(x)
        print("Number of layers in the base model: ", len(self.model.layers))

        self.model = tf.keras.Model(inputs, outputs)

        self.model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])

        # augment the dataset
        # aug_train_x = []
        # aug_train_y = []
        # nim = self.x_train.shape[0]
        # DEGREES_RANGE = 8.
        # SHIFT_RANGE = 2.
        # for i in range(10):
        #     cur_x = np.zeros(self.x_train.shape, dtype=np.uint8)
        #     for j in range(nim):
        #         # plt_showim(self.x_train[j])
        #         rot = DEGREES_RANGE * (random.random() * 2 - 1.)
        #         shiftx = SHIFT_RANGE * (random.random() * 2 - 1.)
        #         shifty = SHIFT_RANGE * (random.random() * 2 - 1.)
        #         cur_x[j] = rotate(self.x_train[j], rot, reshape=False)
        #         cur_x[j] = shift(cur_x[j], (shiftx, shifty))
        #         # plt_showim(cur_x[j])
        #     aug_train_x.append(cur_x)
        #     aug_train_y.append(self.y_train)
        # aug_train_x = np.concatenate(aug_train_x, axis=0)
        # aug_train_y = np.concatenate(aug_train_y, axis=0)
        aug_train_x = np.concatenate((self.x_train, self.x_train_2, self.x_train_3))
        aug_train_y = np.concatenate((self.y_train, self.y_train_2, self.y_train_3))
        aug_train_x = tf.keras.utils.normalize(aug_train_x, axis=1)
        
        aug_train_x_3 = tf.keras.utils.normalize(self.x_train_3, axis=1)
        self.model.fit(aug_train_x_3, self.y_train_3, epochs=10)

        print("TRAINING IS DONE")

    def finetune(self):
        for layer in self.model.layers[:3]:
            layer.trainable = False
        aug_train_x = self.x_train_3
        aug_train_y = self.y_train_3
        aug_train_x = tf.keras.utils.normalize(aug_train_x, axis=1)
        self.model.fit(aug_train_x, aug_train_y, epochs=10)

    def test_data(self):
        print("TESTING THE DATA")

        x_test = tf.keras.utils.normalize(self.x_test, axis=1)
        loss, accuracy = self.model.evaluate(x_test, self.y_test)
        print(f"Loss: {loss}")
        print(f"Accuracy: {accuracy}")

        print("TESTING IS DONE")

    def save_model(self, save):
        if save == "n":
            print("Module not saved")
        else:
            self.model.save("digit_detection.model")
            print("Module saved")

    def load_model(self):
        folder_path = "digit_detection.model"
        self.model = tf.keras.models.load_model(folder_path)

    def detect_digit(self, im):
        im = tf.keras.utils.normalize(im, axis=0)
        prediction = self.model.predict(im[np.newaxis, :, :], verbose=0)
        return np.argmax(prediction)

    def test_png(self):
        image_number = 0
        digit_path = '../plots'
        correct, all = 0, 0
        for path in os.listdir(digit_path):
            if path[-4:] != '.png':
                continue
            strs = path.split('_')
            label = int(strs[0])
            testim = cv2.imread(f"{digit_path}/{path}")[:, :, 0]
            prediction = self.detect_digit(testim)
            if prediction == label:
                correct += 1
            else:
                print(f'true: {label}, predicted: {prediction} path:{path}')
            all += 1
        print(f'accuracy: {correct}/{all}')

