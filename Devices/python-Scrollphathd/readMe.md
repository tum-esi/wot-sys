## Requirements:
* Pyhton 3.7
* Pip3 9.0.1 
&nbsp;


## Installation:
1. Clone the repository in a local file.
2. Go into the that directory in terminal and run ``` pip3 install -r requirements.txt ``` .&nbsp;
3. Run ``` pyhton3 phat.py ```.
## Important Notes:

1. In phat.py, replace TD_DIRECTORY_ADDRESS, if you dont have a directory comment out submit_td().
2. In phat.py, replace the router address you connect.
3. You can get the Thing Description from ``` localhost:port/ ```.
4. You can use other ports as well.
5. Image Properties:
    * File type: Bmp
    * Dimensions: 17x7 Pixels
    * bpp: 8

&nbsp;

## How To Create bmp Image:
* **Using GIMP:**
    * ![readme1](readme_images/gimp1.png)&nbsp;
    * ![readme2](readme_images/gimp2.png)&nbsp;
    * ![readme3](readme_images/gimp3.png)&nbsp;
    * ![readme4](readme_images/gimp4.png)&nbsp;
    * ![readme5](readme_images/gimp5.png)&nbsp;
    * ![readme6](readme_images/gimp6.png)&nbsp;
    * ![readme7](readme_images/gimp7.png)&nbsp;
* **With Hex Editor:**

Refer to this [tutorial](https://medium.com/sysf/bits-to-bitmaps-a-simple-walkthrough-of-bmp-image-format-765dc6857393)

&nbsp;


## Example use:
&nbsp;

* **Uploading Image Using Postman**
&nbsp;

![readme8](readme_images/readme1.png)
![readme9](readme_images/readme2.png)&nbsp;

&nbsp;


* **Uploading Image Using Curl Command**

```
curl --location --request POST 'http://<device ip>:8080/actions/sendImage' --form 'example.bmp=@"<directory of image>/example.bmp"'
```





