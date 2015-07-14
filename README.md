# pyCMVision

pyCMVision is CMVision (color-segmentation library) Python port for cameras with V4L2 drivers.

# Usage

## Get image and print pixel values
```python
import pyCMVision

cam = pyCMVision.Camera()
print(cam.image())
```

## show color-segmented image
```python
import pyCMVision
import numpy as np
import cv2

cam = pyCMVision.Camera()# Open the camera

colors = np.zeros((256,256,256), dtype=np.uint8)
colors[0:128,0:128,20:256] = 1#set colors v=0..128, u=0..128, y=128..255 index to 1
cam.setColors(colors)

cam.setColorMinArea(1, 10)#find blobs having color 1 with min area 30

cam.analyse()

if len(blobs) > 0:
	print('x: {x}, y: {y}'.format(x=blobs[0,3], y=blobs[0,4]))
```

# API

- pyCMVision.Camera(path="/dev/video0", w=640, h=480, fps=30, start=1)
Open camera. Arguments are optional:
*path* is video stream location,
*w* camera width,
*h* camera height,
*fps* framerate,
*start*=1 means camera is started automatically.´
When using start=0 cam.start() must be called manually before capturing images.

```python
import pyCMVision

cam = pyCMVision.Camera()
```

- settings() -> {str name, int value, int default, int min, int max, int step}

Return all available camera settings.
```python
print(cam.settings())
>>>[[name:"brightness", value: 30, default: 30, min: 0, max: 255, step: 1], ...]
```

- set(str param, int value) -> void
Change camera settings.
```python
cam.set("brightness", 50)
```

- get(str param) -> int value
Get camera setting.
```python
print(cam.get("brightness"))
>>> 50
```

- start()
Start video capture.

- stop()
Stop video capture.

- opened() -> bool
Returns true if camera is opened (false if camera is not found).

- started() -> bool
Returns true if video capture is started.

- image(str format) -> nparray [height, width, 3]
Takes image and returns pixel array. Available formats are yuv (default), rgb, bgr.

```python
print(cam.image())
>>> [[255,,0],[y,u,v],...]
print(cam.image("rgb"))
>>> [[255,255,255],[r,g,b],...]
```

- shape() -> (height, width)
Retrieve image dimensions

```python
print(cam.shaoe())
>>> (480, 640)
```

- setColorMinArea(int color_id, int pixels)
Sets minimum blob size for one color.

- setColors(uint8[256][256][256])
Maps every color to color_id.

```python
import numpy as np
colors = np.zeros((256,256,256), dtype=np.uint8)
colors[0:128,:,:] = 1#colors with v (in YUV colorspace) have index 1, everything else have index 0.
cam.setColors(colors)
```

- setPixels(uint8[height][width] active)
Set active pixels. Must have the same shape as image. 1=pixel is active and is used in color segmentation.

```python
import numpy as np
h, w = cam.shape()
ys, xs = np.mgrid[:h,:w]
active_pixels = np.ones((h, w), dtype=np.uint8)
active_pixels[(ys-240)**2+(xs-320)**2 > 100**2] = 0#pixels which are outside the circle r=100 (x=320, y=240) are inactive
cam.setPixels(active_pixels)
```

- setLocations(uint16[height][width] distances, uint16 angles)
Maps each pixel coordinate to polar coordinate (r, phi).

```python
import numpy as np
h, w = cam.shape()
ys, xs = np.mgrid[:h,:w]
distances = (h-ys)*2
phis = (xs - w) * 60 / w
cam.setPixels(distances, phis)
```

- analyse()
Get new frame and run color-segmentation.

- getBuffer() -> nparr buffer [height, width]
Returns segmented image buffer (each pixel color is already mapped to color_id).

```python
import numpy as np

colors = np.zeros((256,256,256), dtype=np.uint8)
colors[0:128,:,:] = 1#colors with v (in YUV colorspace) have index 1, everything else have index 0.
cam.setColors(colors)

segmented_buffer = cam.getBuffer()

cam.analyse()

print(segmented_buffer)
>>> [[0,1,1,1,0,0,...],...]#first row's 2.-4. pixel color v-value is from 0 to 128. 
```

- getBlobs(int color_id) -> nparr [][distance, angle, pixels, x_centroid, y_centroid, x_min, x_max, y_min, y_max]

Returns all blolbs having the same color.

```python
import numpy as np

colors = np.zeros((256,256,256), dtype=np.uint8)
colors[0:128,:,:] = 1#colors with v (in YUV colorspace) have index 1, everything else have index 0.
cam.setColors(colors)

cam.analyse()
cam.setColorMinArea(1, 10)# blob must contain at least 10 pixels

print(cam.getBlobs())
>>> [[0,0,80,320,240,318,324,238,242],...]
```