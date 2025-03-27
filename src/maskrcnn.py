import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
torch.cuda.init()
print("PyTorch version:", torch.__version__)
print("CUDA available:", torch.cuda.is_available())
print("CUDA version:", torch.version.cuda)
print("cuDNN version:", torch.backends.cudnn.version())
print("Device name:", torch.cuda.get_device_name(0))

# Ensure cuDNN is enabled
cudnn.enabled = True
cudnn.benchmark = True

if not torch.cuda.is_available():
    raise RuntimeError("CUDA is not available. Please check your CUDA installation.")

# Create a simple convolution layer
conv_layer = nn.Conv2d(in_channels=3, out_channels=16, kernel_size=3, padding=1).cuda()

# Create a small input tensor to minimize memory issues
input_tensor = torch.randn(1, 3, 32, 32).cuda()

try:
    output = conv_layer(input_tensor)
    print("cuDNN is functioning correctly. Output shape:", output.shape)
except Exception as e:
    print("Error during convolution operation:", e)


"""
import torch
import torchvision
from torchvision import transforms as T
from PIL import Image
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
import time

# load pre-trained model

device = torch.device("cuda")
rospy.loginfo(f"Using device: {device}")
print(torch.cuda.is_available())  
print(torch.cuda.current_device())  
print(torch.cuda.get_device_name(torch.cuda.current_device()))  

model = torchvision.models.detection.maskrcnn_resnet50_fpn(weights='COCO_V1')
model = model.to(device)
model.eval()
bridge = CvBridge()

start_time = time.time()
torch.cuda.empty_cache()

COLOR_MAP = [
    [0, 255, 0], [0, 0, 255], [255, 0, 0], [0, 255, 255], [255, 255, 0],
    [255, 0, 255], [80, 70, 180], [250, 80, 190], [245, 145, 50],
    [70, 150, 250], [50, 190, 190]
]



def deterministic_color_mask(image, idx):
    color = COLOR_MAP[idx % len(COLOR_MAP)]
    r, g, b = np.zeros_like(image), np.zeros_like(image), np.zeros_like(image)
    r[image == 1], g[image == 1], b[image == 1] = color
    rospy.loginfo(f"color {color}")
    return np.stack([r, g, b], axis=2)


def instance_segmentation(img):
    transform = T.Compose([T.ToPILImage(), T.ToTensor()])
    img_tensor = transform(img).unsqueeze(0).to(device)  

    with torch.no_grad():
        output = model(img_tensor)[0]  

    masks = output['masks']  
    img_overlay = img.copy()

    for i, mask in enumerate(masks):
        mask = (mask[0] > 0.5).byte().detach().to(device)  
        color_mask = deterministic_color_mask(mask.cpu().numpy(), i) #mask cpu for overlay
        img_overlay = cv2.addWeighted(img_overlay, 1, color_mask, 0.5, 0)  

    rospy.loginfo(" mask done ")
    rospy.loginfo("--- %s seconds ---" % (time.time() - start_time))

    return img_overlay

# every 3rd frame
frame_count = 0
def image_callback(msg):
    global frame_count
    frame_count += 1
    if frame_count % 3 != 0:
        return

    try:
        
        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        result_img = instance_segmentation(img)
        result_ros_img = bridge.cv2_to_imgmsg(result_img, encoding="bgr8")
        processed_image_pub.publish(result_ros_img)
    except CvBridgeError as e:
        rospy.logerr("CV Bridge Error: %s", e)


rospy.init_node('mask_rcnn_instance_segmentation', anonymous=True)

image_sub = rospy.Subscriber('/camera/color/image_raw', ROSImage, image_callback)
processed_image_pub = rospy.Publisher('/mask_rcnn_output', ROSImage, queue_size=10)
rospy.spin()"
""
""
"""
