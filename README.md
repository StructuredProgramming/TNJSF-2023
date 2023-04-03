These are the code files for my dataset and Machine Learning model for my project for TNJSF, which was completed with the support of PhD Candidate Anar Nurizada and Professor Dr. Anurag Purwar at Stony Brook University. This project creates a model that uses Machine Learning to reconstrut the coupler curves of four-bar, six-bar, and eight-bar mechanisms, while also testing which input representations of these curves yield the best results.

Here is the drive link to the text file of the joint locations of the four-bar, six-bar, and eight-bar linkage systems for the pipeline using Fourier Descriptors, Wavelet Descriptors, and (x,y) points on the curve: https://drive.google.com/file/d/1Ave_nD11YWyKRb4ygaHdYKLut1RlcTtc/view?usp=share_link

Here is the drive link to the images of the synthesized coupler curves of the mechanisms: https://drive.google.com/file/d/1NGsEyE0xRnurBknl1GHvG1hpmQssQSPv/view

Below are images of the matrices I used for each mechanism to denote which joints are connected by links, as well as the different topologies I used for the four-bar, six-bar, and eight-bar linkage system, visualized on the motiongen.io software.

![image](https://user-images.githubusercontent.com/99061771/229416261-6170f0ed-e12c-44e3-8f34-22247ff2b2ee.png)
![image](https://user-images.githubusercontent.com/99061771/229416289-8f3db740-223e-45aa-9dc8-a48c611ed6dd.png)
![image](https://user-images.githubusercontent.com/99061771/229416314-2033064b-961f-48d0-aac1-8e8b167a7ed0.png)
![image](https://user-images.githubusercontent.com/99061771/229416356-7e8518a4-2693-417f-a2f7-b4c4bf77a9a8.png)
![image](https://user-images.githubusercontent.com/99061771/229416493-269366e3-0039-406f-8083-b1cee016f84e.png)
![image](https://user-images.githubusercontent.com/99061771/229416529-68934577-b445-4f1b-ab3d-5bb6b27144c5.png)

These were the final results:

![image](https://user-images.githubusercontent.com/99061771/229412064-3e803cd9-43ca-44b7-9ff8-c9a2540dd2ff.png)

The below images are in order, reconstructed coupler curves using 22 Fourier Descriptors, 44 Wavelet Descriptors, and 360 (x,y) points. Blue denotes the input curve and red is the reconstructed curve:

![image](https://user-images.githubusercontent.com/99061771/229412241-34c1e54d-0653-4406-87bb-285f91fe3d38.png)
![image](https://user-images.githubusercontent.com/99061771/229412420-da8bb82f-5f8f-4cc0-bd13-82e0a8b48932.png)
![image](https://user-images.githubusercontent.com/99061771/229412468-cfab0709-678d-4865-a84e-4f97af8eb72f.png)

Project earned IEEE NJ Section Young Engineer Award ($250), Naval Science Award ($75), and ranked in the top 5 in the Engineering Category.

How to use wavedec to extract wavelet descriptors: https://pywavelets.readthedocs.io/en/latest/ref/dwt-discrete-wavelet-transform.html 

How to extract fourier descriptors (the math behind it): https://www.math.uci.edu/icamp/summer/research_11/bhonsle/Fourier_descriptor.pdf
