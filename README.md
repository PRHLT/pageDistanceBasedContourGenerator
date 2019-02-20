# pageDistanceBasedContourGenerator

Program that calculates the extraction polygon of present text lines given an existing baseline in the page file as per the article that can be found here [Distance Map Article](https://ieeexplore.ieee.org/document/8583787)

## Requirements

- This software requires specific libraries to be present in the system in order to compile: 

1. OpenCV 2.4.9 
2. Boost 1.58.0
3. Eigen3 3.2.92
4. Log4cxx 0.10.0

Although the sofware might run in higher versions of this libraries it has not been tested on them.
Furthermore, it is known that higher versions of OpenCV break backwards compatability with certain
calls used in this software. 

## Compilation

To compile this software simply download this repository, change directory to it and execute:
```
make all
```

## Use of the software

1. The software provides a console parameter menu whith the following command:
```
extract_lines --help
```
2. The software can be run on the provided sample images with the following example command:
```
./extract_lines -i 156730303.jpg -p 156730303.xml -o output.xml -w 4 -x -1  
```

---

## Author
* Vicente Bosch Campos
