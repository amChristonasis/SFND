# SFND 2D Feature Tracking

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## MP.1 Data Buffer Optimization :

If the buffer_size is superior to 3, I'm erase the first value of the buffer_vector.

## MP.2 Keypoint Detection :

I used the Harris and Shi-Tomasi keypoints detector already done in the lesson and I used the function already implemented in OpenCV for the others.

## MP.3 Keypoint Removal :

I'am creat a new keypoint vector and I take only the keypoints include in the rectangle of the car.

## MP.4 Keypoint Descriptors :

I used the function already implemented in OpenCV for this Descriptors.

## MP.5 Descriptor Matching :

I re-used the implementation done previously in the lesson.

## MP.6 Descriptor Distance Ratio :

I re-used the implementation done previously in the lesson.

## MP.7 Performance Evaluation 1 :
				
|Methods |	SHITOMASI |	HARRIS |	FAST |	BRISK |	ORB |	AKAZE |	SIFT |
| - |:-: |:-:|:-:|:-:|:-:|:-:|:-: |
| 1 |	127 |	17 |	149 |	254 |	91 |	162 |	137 | 
|2	|120	|14	|152	|274	|102	|157	|131|
|3	|123	|18	|152	|276	|106	|159	|121|
|4	|120	|21	|157	|275	|113	|154	|136|
|5	|120	|26	|149	|293	|109	|162	|134|
|6	|115	|43	|150	|275	|124	|163	|139|
|7	|114	|18	|157	|289	|129	|173	|136|
|8	|125	|31	|152	|268	|127	|175	|147|
|9	|112	|26	|139	|260	|124	|175	|156|
|10	|113	|34	|144	|250	|125	|175	|135|
|Average	|118,9	|24,8	|150,1	|271,4	|115	|165,5	|137,2|
|Standard deviation	|5,216427045	|9,077444574	|5,466056877	|13,8740365	|12,77149604	|8,195527235	|9,25922963|

## MP.8 Performance Evaluation 2 :

See sheet Project_2_udacity.ods

## MP.8 Performance Evaluation 3 :

Top 3 Detector / Descriptor :
- FAST / ORB
- ORB / BRIEF
- SHI-TOMASI / BRIEF
