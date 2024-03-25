import cv2
import numpy as np
import keras
from tensorflow.keras.models import load_model



def Classify(image):
    # image= cv2.imread( image_path, cv2.COLOR_BGR2RGB)
    dup = image.copy()
    image=cv2.resize(image, (224,224),interpolation = cv2.INTER_AREA)
    image=np.array(image)
    image = image.astype('float32')
    # image /= 255 
    image = np.reshape(image, (1, 224,224,3))
    #keras.config.enable_unsafe_deserialization()
   
    fmodel = load_model("/home/jermito/rina_ws/src/rina_bot/cnn/Fmodel.h5")
    Prediction = fmodel.predict(image)
    index = np.argmax(Prediction)
    if index == 0:
        disease = "Potato_EarlyBlight"
    elif index == 2:
        disease = "Potato_Healthy"
    else:
        disease = "Potato_LateBlight"
    image = dup
    cv2.putText(image, disease, (0,470), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, 2)
    cv2.imshow(disease,image)
    cv2.waitKey(5000)
    cv2.destroyAllWindows()
    return Prediction, np.argmax(Prediction), disease





   
