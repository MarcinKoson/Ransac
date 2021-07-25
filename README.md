# Searching for similar objects using RANSAC 

Implementation of RANSAC for finding similar objects on two pictures.

GUI for this library
https://github.com/MarcinKoson/RansacGUI

## Usage

As an input application use haraff sift files. 
You can get generate it for .png image using 
[www.robots.ox.ac.uk/~vgg/research/affine/det_eval_files/extract_features2.tar.gz]()

Generate haraff sift file (Linux)  
`./extract_features_32bit.ln -haraff -sift -i yourpicture.png`  
Images with high resolution may cause crashes.

There is one function

`string Start(string file1, string file2,int sizeN,int cohesionN, int iterations, int maxError, int afiOrPersp)`

### Parameters
* file1
	>Name of first .haraff.sift file
* file2
	>Name of second .haraff.sift file
* sizeN  
	>Size of neighborhood of pair.
* cohesionN  
	>Specifies how many pairs must be in neighborhood of each point of pair to accept pair as consistent. Inconsistent pairs are discarded. Should be smaller than size of neighborhood.
* iterations  
	>Number of repetitions of model calculation in RANSAC.
* maxError 
	>Maximum error of the pair to be determined as matching the model.
* afiOrPersp
	>Pass '3' for affine transformation, pass '4' for perspective transformation

### Return
Function returns string in format

`result-KeyPointPairs-ConsistentPairs-RansacPairs/x1;y1;x2;y2/x1;y1;x2;y2/ ... / x1;y1;x2;y2/`

* result 
	>result of function  
	>100 - Success    
	>200 - Incorrect first .haraff.sift file  
	>201 - Incorrect second .haraff.sift file  
	>202 - Error during pairing  
	>203 - Error with cohesion calculation  
	>204 - Error with RANSAC  	
* KeyPointPairs
	>number of key point pairs	
* ConsistentPairs  
	>number of consistent pairs	
* RansacPairs  
	>number of ransac pairs	
* x1  
	>X coordinate on first picture	
* y1 
	>Y coordinate on first picture	
* x2
	>X coordinate on second picture	
* y2
	>Y coordinate on second picture

## Technologies
C#


