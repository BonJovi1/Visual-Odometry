# Visual-Odometry
Feature based visual odometry 

## Instructions

To install Jupyter Notebook, if you don't have it, run
`pip install jupyter` \
And then, run the command `jupyter-notebook code.ipynb`

## The Question

Visual odometry (VO) is the process of recovering the egomotion (in other words, the trajectory) of an agent using only the input of a camera or a system of cameras attached to the agent. This is a well- studied problem in robotic vision and is a critical part of many applications such as mars rovers, and self-driving cars for localization. You will be implementing a basic monocular visual odometry algorithm in this part of the assignment.

To begin with, download all the required files from [here]. It contains a sequence of images from the KITTI dataset. The ground truth pose of each frame (in row-major order) and the camera parameters are provided as well.
We will now go through the procedure step-by-step. The following is an overview of the entire algorithm,

1. Find corresponding features between frames Ik, Ik−1.
2. Using these feature correspondences, estimate the essential matrix between the two images within
a RANSAC scheme.
3. Decompose this essential matrix to obtain the relative rotation Rk and translation tk, and form
the transformation Tk.
4. Scale the translation tk with the absolute or relative scale.
5. Concatenate the relative transformation by computing Ck = Ck−1Tk, where Ck−1 is the previous pose of the camera in the world frame.
6. Repeat steps 1 − 5 for the remaining pairs of frames.

The main task in VO is to compute the relative transformations Tk from each pair of images Ik and Ik−1 and then to concatenate these transformations to recover the full trajectory C0:n of the camera, where n is the total number of images. C0 is taken to be the origin i.e. the world frame. There are two broad approaches to compute the relative motion Tk: appearance-based (or direct) methods, which use the intensity information of all the pixels in the two input images, and feature-based methods, which only use salient and repeatable features extracted and tracked across the images. You will be implementing a feature-based method.

For every new image Ik, the first step consists of detecting and matching 2D features with those from the previous frame. These 2D features (or simply keypoints) are locations in the image which we can reliably find in multiple images and possibly match them. To detect these keypoints use the following OpenCV code.

`detector = cv2.SIFT() <br/>
keypoints = detector.detect(img1) <br/>
pts1 = np.array([x.pt for x in keypoints], dtype=np.float32)`

SIFT (scale invariant feature detector) is one of many feature detectors, which applies a difference-of- Gaussian (DoG) operator on the entire image, followed by a nonmaxima supression on its output to detect the features. It achieves scale invariance by applying the detector on lower-scale and upper-scale versions of the image. You are not expected to know all the details of SIFT here.

Every detected keypoint is then associated with a description of the neighborhood it belongs to, which is called a descriptor. These descriptors are then used for searching for corresponding features in other images based on a similarity measure.
An alternative way to independently finding features in all candidate images and then matching them is to use a detect-then-track approach. Features are detected in the first image, and then tracked over the next set of images. For this, use OpenCV’s Lukas-Kanade tracker.

`pts2, status = cv2.calcOpticalFlowPyrLK(img1,img2,pts1) <br/>
pts1 = pts1[status == 1] <br/>
pts2 = pts2[status == 1]`

The function computes the location of the points from the first image in the second image, by computing their ’optical flow’, or simply their apparent motion. This optical flow is computed by applying the Lukas-Kande algorithm, an algorithm that uses spatial and temporal image gradients to compute the motion of the points (hence their locations). It also makes the assumption that nearby point have the same motion. Note that some features will eventually move out of the field-of-view, and tracks will be lost, so make sure to detect new features when the number of features goes below a threshold (say, 150).

As mentioned earlier, the main task is motion computation. Using these feature correspondences, imple- ment the 8-point algorithm for fundamental matrix estimation. Implement it inside a RANSAC scheme to get rid of any outliers, as explained in class. Then, compute the essential matrix, and decompose it to the relative R and t using cv2.recoverPose(E, points1, points2, K, R, t[, mask]). Note that the function returns the R and t of the first camera with respect to the second, and not the other way around (The joys of working in robotics :’)).

Now, you might recall that the absolute scale of the translation cannot be computed from just two images. The above function only returns the direction of t, as a unit vector. Use the ground truth translation to get the absolute scale, and multiply your unit translation with this scale. Then concatenate your transformations, and repeat for the next pair of frames to recover the full absolute trajectory.
