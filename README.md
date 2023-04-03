These are the code files for my dataset and Machine Learning model for my project for TNJSF. This project creates a model that uses Machine Learning to reconstrut the coupler curves of four-bar, six-bar, and eight-bar mechanisms, while also testing which input representations of these curves yield the best results.

Here is the drive link to the text file of the joint locations of the four-bar, six-bar, and eight-bar linkage systems for the pipeline using Fourier Descriptors, Wavelet Descriptors, and (x,y) points on the curve: https://drive.google.com/file/d/1Ave_nD11YWyKRb4ygaHdYKLut1RlcTtc/view?usp=share_link

Here is the drive link to the images of the synthesized coupler curves of the mechanisms: https://drive.google.com/file/d/1NGsEyE0xRnurBknl1GHvG1hpmQssQSPv/view

![image](https://user-images.githubusercontent.com/99061771/229412064-3e803cd9-43ca-44b7-9ff8-c9a2540dd2ff.png)

The below images are in order, reconstructed coupler curves using 22 Fourier Descriptors, 44 Wavelet Descriptors, and 360 (x,y) points. Blue denotes the input curve adn red is the reconstructed curve.

![image](https://user-images.githubusercontent.com/99061771/229412241-34c1e54d-0653-4406-87bb-285f91fe3d38.png)
![image](https://user-images.githubusercontent.com/99061771/229412420-da8bb82f-5f8f-4cc0-bd13-82e0a8b48932.png)
![image](https://user-images.githubusercontent.com/99061771/229412468-cfab0709-678d-4865-a84e-4f97af8eb72f.png)

How to use wavedec to extract wavelet descriptors: https://pywavelets.readthedocs.io/en/latest/ref/dwt-discrete-wavelet-transform.html 

How to extract fourier descriptors (the math behind it): https://www.math.uci.edu/icamp/summer/research_11/bhonsle/Fourier_descriptor.pdf
