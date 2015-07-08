// python v4l2 functions taken from python-v4l2capture module
// (https://github.com/gebart/python-v4l2capture)

#define USE_LIBV4L

#include <Python.h>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include "numpy/arrayobject.h"
#ifdef USE_LIBV4L
#include <libv4l2.h>
#else
#include <sys/ioctl.h>
#define v4l2_close close
#define v4l2_ioctl ioctl
#define v4l2_mmap mmap
#define v4l2_munmap munmap
#define v4l2_open open
#endif

#ifndef Py_TYPE
	#define Py_TYPE(ob) (((PyObject*)(ob))->ob_type)
#endif


#define ASSERT_OPEN if(self->fd < 0) { \
	PyErr_SetString(PyExc_ValueError, "I/O operation on closed file"); \
	return NULL; \
}

#define CLEAR(x) memset(&(x), 0, sizeof(x))

struct buffer {
	void *start;
	size_t length;
};

#define MAX_WIDTH 1280
#define MAX_HEIGHT 1024
#define MAX_INT 2147483647
#define COLOR_COUNT 10
#define CMV_RBITS 6
#define CMV_RADIX (1 << CMV_RBITS)
#define CMV_RMASK (CMV_RADIX-1)
#define MAX_RUNS MAX_WIDTH * MAX_HEIGHT / 4
#define MAX_REG MAX_WIDTH * MAX_HEIGHT / 16

#define max(a,b) \
	({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a > _b ? _a : _b; })
	 
#define min(a,b) \
	({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a < _b ? _a : _b; })

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

typedef struct {
	short x, y, width;
	unsigned char color;
	int parent, next;
} run;

typedef struct region {
	int color;
	int x1, y1, x2, y2;
	float cen_x, cen_y;
	int area;
	int run_start;
	int iterator_id;
	struct region* next;
} region;

typedef struct {
	region *list;
	int num;
	int min_area;
	unsigned char color;
	char *name;
} color_class_state;

typedef struct {
	char* keyword;
	int id;
} v4l2_settings_t;

typedef struct {
	PyObject_HEAD
	int fd;
	struct buffer *buffers;
	int buffer_count;
	unsigned char colors_lookup[0x1000000];//all possible bgr combinations lookup table
	unsigned short loc_r[MAX_WIDTH * MAX_HEIGHT];//pixel location to distance lookup table
	unsigned short loc_phi[MAX_WIDTH * MAX_HEIGHT];//pixel location to angle lookup table
	unsigned char pixel_active[MAX_WIDTH * MAX_HEIGHT];//0=ignore in segmentation, 1=use pixel
	unsigned char *segmented;//segmented image buffer 0-9
	unsigned char *img;//Image buffer
	unsigned short *pout;//Temp out buffer (for blobs)
	int width, height, bpp;
	unsigned char started;

	int v4l2_settings_n;
	v4l2_settings_t v4l2_settings[256];
	
	run rle[MAX_RUNS];
	region regions[MAX_REG];
	color_class_state colors[COLOR_COUNT];
	int run_c;
	int region_c;
	int max_area;
	int passes;
} Camera;

static int my_ioctl(int fd, int request, void *arg) {
	// Retry ioctl until it returns without being interrupted.
	for(;;) {
		int result = v4l2_ioctl(fd, request, arg);
		if(!result) {
			return 0;
		}
		if(errno != EINTR) {
			PyErr_SetFromErrno(PyExc_IOError);
			return 1;
		}
	}
}

static void Camera_unmap(Camera *self) {
	int i;

	for(i = 0; i < self->buffer_count; i++) {
		v4l2_munmap(self->buffers[i].start, self->buffers[i].length);
	}
}

static void Camera_dealloc(Camera *self) {
	if(self->fd >= 0) {
		if(self->buffers) {
			Camera_unmap(self);
		}

		v4l2_close(self->fd);
	}
	if (self->segmented != NULL) {
		free(self->segmented);
	}
	if (self->img != NULL) {
		free(self->img);
	}
	if (self->pout != NULL) {
		free(self->pout);
	}

	Py_TYPE(self)->tp_free((PyObject *)self);
}

static int Camera_set_resolution(Camera *self, int w, int h) {
	struct v4l2_format format;
	CLEAR(format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	/* Get the current format */
	if(my_ioctl(self->fd, VIDIOC_G_FMT, &format)) {
		return -1;
	}

	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	format.fmt.pix.field = V4L2_FIELD_INTERLACED;
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = w;
	format.fmt.pix.height = h;
	format.fmt.pix.bytesperline = 0;

	if(my_ioctl(self->fd, VIDIOC_S_FMT, &format)) {
		return -1;
	}

	self->width = (int)w;
	self->height = (int)h;
	
	int size = self->width * self->width;
	
	if (self->segmented != NULL) {
		free(self->segmented);
	}
	self->segmented = (unsigned char *)calloc(size, sizeof(unsigned char));
	
	if (self->img != NULL) {
		free(self->img);
	}
	self->img = (unsigned char *)malloc(size * sizeof(unsigned char) * 3);
	return 0;
}

static int Camera_set_fps(Camera *self, int fps) {
	struct v4l2_streamparm setfps;
	CLEAR(setfps);
	setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	setfps.parm.capture.timeperframe.numerator = 1;
	setfps.parm.capture.timeperframe.denominator = fps;
	if(my_ioctl(self->fd, VIDIOC_S_PARM, &setfps)){
		return -1;
	}
	return 0;
}

static int Camera_create_buffers(Camera *self, int buffer_count) {

	if(self->fd < 0) {
		return -1;
	}

	if(self->buffers) {
		//PyErr_SetString(PyExc_ValueError, "Buffers are already created");
		return -1;
	}

	struct v4l2_requestbuffers reqbuf;
	reqbuf.count = buffer_count;
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_MMAP;

	if(my_ioctl(self->fd, VIDIOC_REQBUFS, &reqbuf)) {
		return -1;
	}

	if(!reqbuf.count) {
		//PyErr_SetString(PyExc_IOError, "Not enough buffer memory");
		return -1;
	}

	self->buffers = malloc(reqbuf.count * sizeof(struct buffer));

	if(!self->buffers) {
		//PyErr_NoMemory();
		return -1;
	}

	int i;

	for(i = 0; i < reqbuf.count; i++) {
		struct v4l2_buffer buffer;
		buffer.index = i;
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;

		if(my_ioctl(self->fd, VIDIOC_QUERYBUF, &buffer)) {
			return -1;
		}

		self->buffers[i].length = buffer.length;
		self->buffers[i].start = v4l2_mmap(NULL, buffer.length,
			PROT_READ | PROT_WRITE, MAP_SHARED, self->fd, buffer.m.offset);

		if(self->buffers[i].start == MAP_FAILED) {
			//PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
	}

	self->buffer_count = i;
	//Py_RETURN_NONE;
	return 0;
}

static int Camera_queue_all_buffers(Camera *self) {
	if(!self->buffers) {
		//ASSERT_OPEN;
		//PyErr_SetString(PyExc_ValueError, "Buffers have not been created");
		return -1;
	}

	int i;
	int buffer_count = self->buffer_count;

	for(i = 0; i < buffer_count; i++) {
		struct v4l2_buffer buffer;
		buffer.index = i;
		buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buffer.memory = V4L2_MEMORY_MMAP;

		if(my_ioctl(self->fd, VIDIOC_QBUF, &buffer)) {
			return -1;
		}
	}

	return 0;
}

static int Camera_init(Camera *self, PyObject *args, PyObject *kwargs) {
	static char *kwlist [] = {
		"path",
		"w",
		"h",
		"fps",
		NULL
	};
	const char *device_path;
	device_path = "/dev/video0";
	int w = 640;
	int h = 480;
	int fps = 30;

	if(!PyArg_ParseTupleAndKeywords(args, kwargs, "|siii", kwlist, &device_path, &w, &h, &fps)) {
		return -1;
	}

	int fd = v4l2_open(device_path, O_RDWR | O_NONBLOCK);

	if(fd < 0) {
		PyErr_SetFromErrnoWithFilename(PyExc_IOError, (char *)device_path);
		return -1;
	}

	self->fd = fd;
	self->buffers = NULL;

	self->started = 0;
	self->bpp = 1;
	self->width = 0;
	self->height = 0;
	self->segmented = NULL;
	self->img = NULL;
	self->pout = (unsigned short *) malloc(10000 * 9 * sizeof(unsigned short));
	self->run_c = 0;
	self->region_c = 0;
	self->max_area = 0;
	
	int i;
	for(i=0; i<COLOR_COUNT; i++) {
		self->colors[i].list = NULL;
		self->colors[i].num	= 0;
		self->colors[i].min_area = MAX_INT;
		self->colors[i].color = i;
	}
	
	for (i=0; i<MAX_WIDTH * MAX_HEIGHT; i++) {
		self->pixel_active[i] = 1;
	}

	Camera_set_resolution(self, w, h);
	Camera_set_fps(self, fps);
	Camera_create_buffers(self, 3);
	Camera_queue_all_buffers(self);

	//load v4l2 settings parameters. Check does camera supports these?
	v4l2_settings_t settings[] = {
		{"exposure_auto", V4L2_CID_EXPOSURE_AUTO},
		{"exposure_absolute", V4L2_CID_EXPOSURE_ABSOLUTE},
		{"white_balance_automatic", V4L2_CID_AUTO_WHITE_BALANCE},
		{"red_balance", V4L2_CID_RED_BALANCE},
		{"green_balance", V4L2_CID_GAMMA},
		// V4L2 dont have green balance settings
		// only useful with PS3 Eye camera and modified ov534 driver
		// https://github.com/lwd8cmd/Mitupead/blob/master/drivers/ov534.c
		{"blue_balance", V4L2_CID_BLUE_BALANCE},
		{"gain_automatic", V4L2_CID_AUTOGAIN},
		{"brightness", V4L2_CID_BRIGHTNESS},
		{"contrast", V4L2_CID_CONTRAST},
		{"saturation", V4L2_CID_SATURATION},
		{"hue", V4L2_CID_HUE},
		{"gain", V4L2_CID_GAIN},
		{"sharpness", V4L2_CID_SHARPNESS},
		{"vertical_flip", V4L2_CID_VFLIP},
		{"horizontal_flip", V4L2_CID_HFLIP},
		{"white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE},
		// Definition overlapping green_balance
		{"gamma", V4L2_CID_GAMMA},
		{"power_line_frequency", V4L2_CID_POWER_LINE_FREQUENCY},
		{"backlight_compensation", V4L2_CID_BACKLIGHT_COMPENSATION},
		{"pan_absolute", V4L2_CID_PAN_ABSOLUTE},
		{"tilt_absolute", V4L2_CID_TILT_ABSOLUTE},
	};
	self->v4l2_settings_n = ARRAY_SIZE(settings);
	for (i=0; i<self->v4l2_settings_n; i++) {
		self->v4l2_settings[i] = settings[i];
	}

	return 0;
}

static int Camera_get_V4L2_CID(Camera *self, int id) {
	struct v4l2_control ctrl;
	CLEAR(ctrl);
	ctrl.id		= id;
	if(my_ioctl(self->fd, VIDIOC_G_CTRL, &ctrl)){
		return -1;
	}
	return ctrl.value;
}

static PyObject *Camera_get_params(Camera *self, PyObject *args) {
	char *param;
	if (!PyArg_ParseTuple(args, "s", &param)) {
		return NULL;
	}

	int i;
	int retval = -1;
	for (i=0; i<self->v4l2_settings_n; i++) {
		if (strcmp(param, self->v4l2_settings[i].keyword) == 0) {
			retval = Camera_get_V4L2_CID(self, self->v4l2_settings[i].id);
			break;
		}
	}
	
	return Py_BuildValue("i", retval);
}

static void Camera_set_V4L2_CID(Camera *self, int id, int value) {
	struct v4l2_control ctrl;
	CLEAR(ctrl);
	ctrl.id		= id;
	ctrl.value	= value;
	my_ioctl(self->fd, VIDIOC_S_CTRL, &ctrl);
}

static PyObject *Camera_set_params(Camera *self, PyObject *args, PyObject *keywds) {
	int maxprms = self->v4l2_settings_n;
	int n = maxprms;
	char *kwlist[n+1];
	int ids[n];
	int vs[n];
	char format[n+1];

	kwlist[n] = NULL;
	format[0] = '|';

	while (n--) {
		kwlist[n] = self->v4l2_settings[n].keyword;
		ids[n] = self->v4l2_settings[n].id;
		vs[n] = -1;
		format[n+1] = 'i';
	}
	
	if (!PyArg_ParseTupleAndKeywords(args, keywds, format, kwlist,
		&vs[0], &vs[1], &vs[2], &vs[3], &vs[4], &vs[5], &vs[6],
		&vs[7], &vs[8], &vs[9], &vs[10], &vs[11], &vs[12], &vs[13], &vs[14])) {
		return NULL;
	}

	n = maxprms;
	while (n--) {
		if (vs[n] > -1) {
			Camera_set_V4L2_CID(self, ids[n], vs[n]);
		}
	}

	Py_RETURN_NONE;
}

static PyObject *CameraShape(Camera *self) {
	//return tuple (height, width)
	return Py_BuildValue("(ii)", self->height, self->width);
}

static PyObject *Camera_start(Camera *self) {
	ASSERT_OPEN;
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if(my_ioctl(self->fd, VIDIOC_STREAMON, &type)) {
		return NULL;
	}
	self->started = 1;

	Py_RETURN_NONE;
}

static PyObject *Camera_stop(Camera *self) {
	ASSERT_OPEN;
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if(my_ioctl(self->fd, VIDIOC_STREAMOFF, &type)) {
		return NULL;
	}

	Py_RETURN_NONE;
}

static PyObject *CameraOpened(Camera *self) {
	//camera selected? return bool
	return Py_BuildValue("b", !(self->fd < 0));
}
static PyObject *CameraStarted(Camera *self) {
	//camera started? return bool
	return Py_BuildValue("b", self->started);
}

static struct v4l2_buffer Camera_fill_buffer(Camera *self) {
	struct v4l2_buffer buffer;
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	buffer.index = -1;
	if(!self->buffers) {
		return buffer;
	}

	fd_set fds;
	struct timeval tv;
	 
	FD_ZERO(&fds);
	FD_SET(self->fd, &fds);
	 
	tv.tv_sec = 2;//timeout
	tv.tv_usec = 0;

	while (select(self->fd + 1, &fds, NULL, NULL, &tv) <= 0);//wait until buffer is filled

	if(my_ioctl(self->fd, VIDIOC_DQBUF, &buffer)) {
		return buffer;
	}
	return buffer;
}

static PyObject *Camera_read_internal(Camera *self, int queue) {
	struct v4l2_buffer buffer = Camera_fill_buffer(self);
	if (buffer.index == -1) {
		return NULL;
	}

	int w = self->width;
	int h = self->height;
	int xy;
	unsigned char* f = (unsigned char*)self->buffers[buffer.index].start;

	for (xy = 0; xy < w*h; xy+=2) {
		int y1, y2, u, v;
		y1 = f[2*xy];
		u = f[2*xy+1];
		y2 = f[2*xy+2];
		v = f[2*xy+3];
		self->img[3*xy] = y1;
		self->img[3*xy+1] = u;
		self->img[3*xy+2] = v;
	self->img[3*xy+3] = y2;
	self->img[3*xy+4] = u;
	self->img[3*xy+5] = v;
	}


	if(queue && my_ioctl(self->fd, VIDIOC_QBUF, &buffer)) {
		return NULL;
	}

	npy_intp dims[3] = {self->height, self->width, 3};
	PyArrayObject *outArray = (PyArrayObject *) PyArray_SimpleNewFromData(3, dims, NPY_UINT8, self->img);
	return PyArray_Return(outArray);
}

static PyObject *Camera_read(Camera *self) {
	return Camera_read_internal(self, 1);
}

static PyObject *CameraSetColorMinArea(Camera *self, PyObject *args) {
	//set min blob size
	int color;
	int min_area;

	if (!PyArg_ParseTuple(args, "ii", &color, &min_area)) {
		return NULL;
	}
	if (color < COLOR_COUNT) {
		self->colors[color].min_area = min_area;
	}
	
	Py_RETURN_NONE;
}

static PyObject *CameraSetColors(Camera *self, PyObject *args) {
	//set colortable
	PyObject *arg1=NULL;
	PyArrayObject *lookup=NULL;

	if (!PyArg_ParseTuple(args, "O!", &PyArray_Type, &arg1)) return NULL;
	lookup = (PyArrayObject*)PyArray_FROM_OTF(arg1, NPY_UINT8, NPY_ARRAY_IN_ARRAY);
	if (lookup == NULL) {
		Py_XDECREF(lookup);
		return NULL;
	}
	
	unsigned char *data = (unsigned char*)PyArray_DATA(lookup);
	unsigned long size = min(0x1000000, (unsigned long)PyArray_NBYTES(lookup));
	memcpy(self->colors_lookup, data, size);
	
	Py_DECREF(lookup);
	Py_RETURN_NONE;
}

static PyObject *CameraSetActivePixels(Camera *self, PyObject *args) {
	//set colortable
	PyObject *arg1=NULL;
	PyArrayObject *pixels=NULL;

	if (!PyArg_ParseTuple(args, "O!", &PyArray_Type, &arg1)) return NULL;
	pixels = (PyArrayObject*)PyArray_FROM_OTF(arg1, NPY_UINT8, NPY_ARRAY_IN_ARRAY);
	if (pixels == NULL) {
		Py_XDECREF(pixels);
		return NULL;
	}
	
	unsigned char *data = (unsigned char*)PyArray_DATA(pixels);
	unsigned long size = min(MAX_WIDTH * MAX_HEIGHT, (unsigned long)PyArray_NBYTES(pixels));
	memcpy(self->pixel_active, data, size);
	
	Py_DECREF(pixels);
	Py_RETURN_NONE;
}

static PyObject *CameraSetLocations(Camera *self, PyObject *args) {
	//set colortable
	PyObject *arg1=NULL, *arg2=NULL;
	PyArrayObject *d_r=NULL, *d_phi=NULL;

	if (!PyArg_ParseTuple(args, "O!O!", &PyArray_Type, &arg1, &PyArray_Type, &arg2)) return NULL;
	d_r = (PyArrayObject*)PyArray_FROM_OTF(arg1, NPY_UINT16, NPY_ARRAY_IN_ARRAY);
	if (d_r == NULL) {
		Py_XDECREF(d_r);
		return NULL;
	}
	d_phi = (PyArrayObject*)PyArray_FROM_OTF(arg2, NPY_UINT16, NPY_ARRAY_IN_ARRAY);
	if (d_phi == NULL) {
		Py_XDECREF(d_phi);
		return NULL;
	}
	
	unsigned short *data_r = (unsigned short*)PyArray_DATA(d_r);
	unsigned long size_r = min(MAX_WIDTH * MAX_HEIGHT * sizeof(unsigned short), (unsigned long)PyArray_NBYTES(d_r));
	memcpy(self->loc_r, data_r, size_r);
	
	unsigned short *data_phi = (unsigned short*)PyArray_DATA(d_phi);
	unsigned long size_phi = min(MAX_WIDTH * MAX_HEIGHT * sizeof(unsigned short), (unsigned long)PyArray_NBYTES(d_phi));
	memcpy(self->loc_phi, data_phi, size_phi);
	
	Py_DECREF(d_r);
	Py_DECREF(d_phi);
	Py_RETURN_NONE;
}

static void SegEncodeRuns(Camera *self) {
// Changes the flat array version of the thresholded image into a run
// length encoded version, which speeds up later processing since we
// only have to look at the points where values change.
	unsigned char m, save;
	unsigned char *row = NULL;
	int x, y, j, l;
	run r;
	unsigned char *map = self->segmented;
	run *rle = self->rle;
	
	int w = self->width;
	int h = self->height;

	r.next = 0;

	// initialize terminator restore
	save = map[0];

	j = 0;
	for(y = 0; y < h; y++){
		row = &map[y * w];

		// restore previous terminator and store next
		// one in the first pixel on the next row
		row[0] = save;
		save = row[w];
		row[w] = 255;
		
		r.y = y;

		x = 0;
		while(x < w){
			m = row[x];
			r.x = x;

			l = x;
			while(row[x] == m) x++;

			if(self->colors[m].min_area < MAX_INT || x >= w ) {
				r.color = m;
				r.width = x - l;
				r.parent = j;
				rle[j++] = r;

				if(j >= MAX_RUNS) {
					row[w] = save;
					self->run_c = j;
					return;
				}
			}
		}
	}

	self->run_c = j;
}

static void SegConnectComponents(Camera *self) {
// Connect components using four-connecteness so that the runs each
// identify the global parent of the connected region they are a part
// of.	It does this by scanning adjacent rows and merging where
// similar colors overlap.	Used to be union by rank w/ path
// compression, but now it just uses path compression as the global
// parent index, a simpler rank bound in practice.
// WARNING: This code is complicated.	I'm pretty sure it's a correct
//	 implementation, but minor changes can easily cause big problems.
//	 Read the papers on this library and have a good understanding of
//	 tree-based union find before you touch it
	int l1, l2;
	run r1, r2;
	int i, j, s;
	int num = self->run_c;
	run *map = self->rle;

	// l2 starts on first scan line, l1 starts on second
	l2 = 0;
	l1 = 1;
	while(map[l1].y == 0) l1++; // skip first line

	// Do rest in lock step
	r1 = map[l1];
	r2 = map[l2];
	s = l1;
	while(l1 < num){
		if(r1.color==r2.color && self->colors[r1.color].min_area < MAX_INT){
			if((r2.x<=r1.x && r1.x<r2.x+r2.width) || (r1.x<=r2.x && r2.x<r1.x+r1.width)){
				if(s != l1){
					// if we didn't have a parent already, just take this one
					map[l1].parent = r1.parent = r2.parent;
					s = l1;
				} else if(r1.parent != r2.parent) {
					// otherwise union two parents if they are different

					// find terminal roots of each path up tree
					i = r1.parent;
					while(i != map[i].parent) i = map[i].parent;
					j = r2.parent;
					while(j != map[j].parent) j = map[j].parent;

					// union and compress paths; use smaller of two possible
					// representative indicies to preserve DAG property
					if(i < j) {
						map[j].parent = i;
						map[l1].parent = map[l2].parent = r1.parent = r2.parent = i;
					} else {
						map[i].parent = j;
						map[l1].parent = map[l2].parent = r1.parent = r2.parent = j;
					}
				}
			}
		}

		// Move to next point where values may change
		i = (r2.x + r2.width) - (r1.x + r1.width);
		if(i >= 0) r1 = map[++l1];
		if(i <= 0) r2 = map[++l2];
	}

	// Now we need to compress all parent paths
	for(i=0; i<num; i++){
		j = map[i].parent;
		map[i].parent = map[j].parent;
	}
}

inline int range_sum(int x, int w) {
	//foo bar
	return(w*(2*x + w-1) / 2);
}

static void SegExtractRegions(Camera *self) {
// Takes the list of runs and formats them into a region table,
// gathering the various statistics along the way.	num is the number
// of runs in the rmap array, and the number of unique regions in
// reg[] (bounded by max_reg) is returned.	Implemented as a single
// pass over the array of runs.
	int b, i, n, a;
	int num = self->run_c;
	run *rmap = self->rle;
	region *reg = self->regions;
	run r;
	n = 0;

	for(i=0; i<num; i++){
		if( self->colors[rmap[i].color].min_area < MAX_INT){
			r = rmap[i];
			if(r.parent == i){
				// Add new region if this run is a root (i.e. self parented)
				rmap[i].parent = b = n;	// renumber to point to region id
				reg[b].color = r.color;
				reg[b].area = r.width;
				reg[b].x1 = r.x;
				reg[b].y1 = r.y;
				reg[b].x2 = r.x + r.width;
				reg[b].y2 = r.y;
				reg[b].cen_x = range_sum(r.x,r.width);
				reg[b].cen_y = r.y * r.width;
				reg[b].run_start = i;
				reg[b].iterator_id = i; // temporarily use to store last run
				n++;
				if(n >= MAX_REG) {
					printf( "Regions buffer exceeded.\n" );
					self->region_c = MAX_REG;
					return;
				}
			} else {
				// Otherwise update region stats incrementally
				b = rmap[r.parent].parent;
				rmap[i].parent = b; // update parent to identify region id
				reg[b].area += r.width;
				reg[b].x2 = max(r.x + r.width,reg[b].x2);
				reg[b].x1 = min((int)r.x,reg[b].x1);
				reg[b].y2 = r.y; // last set by lowest run
				reg[b].cen_x += range_sum(r.x,r.width);
				reg[b].cen_y += r.y * r.width;
				// set previous run to point to this one as next
				rmap[reg[b].iterator_id].next = i;
				reg[b].iterator_id = i;
			}
		}
	}

	// calculate centroids from stored sums
	for(i=0; i<n; i++){
		a = reg[i].area;
		reg[i].cen_x = (float)reg[i].cen_x / a;
		reg[i].cen_y = (float)reg[i].cen_y / a;
		rmap[reg[i].iterator_id].next = 0; // -1;
		reg[i].iterator_id = 0;
		reg[i].x2--; // change to inclusive range
	}
	self->region_c = n;
}

static void SegSeparateRegions(Camera *self) {
// Splits the various regions in the region table a separate list for
// each color.	The lists are threaded through the table using the
// region's 'next' field.	Returns the maximal area of the regions,
// which can be used later to speed up sorting.
	region *p = NULL;
	int i;
	int c;
	int area;
	int num = self->region_c;
	region *reg = self->regions;
	color_class_state *color = self->colors;

	// clear out the region list head table
	for(i=0; i<COLOR_COUNT; i++) {
		color[i].list = NULL;
		color[i].num	= 0;
	}
	// step over the table, adding successive
	// regions to the front of each list
	self->max_area = 0;
	for(i=0; i<num; i++){
		p = &reg[i];
		c = p->color;
		area = p->area;

		if(area >= color[c].min_area){
			if(area > self->max_area) self->max_area = area;
			color[c].num++;
			p->next = color[c].list;
			color[c].list = p;
		}
	}
}

region* SegSortRegions( region *list, int passes ) {
// Sorts a list of regions by their area field.
// Uses a linked list based radix sort to process the list.
	region *tbl[CMV_RADIX]={NULL}, *p=NULL, *pn=NULL;
	int slot, shift;
	int i, j;

	// Handle trivial cases
	if(!list || !list->next) return(list);

	// Initialize table
	for(j=0; j<CMV_RADIX; j++) tbl[j] = NULL;

	for(i=0; i<passes; i++){
		// split list into buckets
		shift = CMV_RBITS * i;
		p = list;
		while(p){
			pn = p->next;
			slot = ((p->area) >> shift) & CMV_RMASK;
			p->next = tbl[slot];
			tbl[slot] = p;
			p = pn;
		}

		// integrate back into partially ordered list
		list = NULL;
		for(j=0; j<CMV_RADIX; j++){
			p = tbl[j];
			tbl[j] = NULL; // clear out table for next pass
			while(p){
				pn = p->next;
				p->next = list;
				list = p;
				p = pn;
			}
		}
	}

	return(list);
}

static PyObject *CameraAnalyse(Camera *self) {
	struct v4l2_buffer buffer = Camera_fill_buffer(self);
	if (buffer.index == -1) {
		return NULL;
	}

	int w = self->width;
	int h = self->height;
	int xy;
	unsigned char* f = (unsigned char*)self->buffers[buffer.index].start;
	for (xy = 0; xy < w*h; xy+=2) {
		int y1, y2, u, v;
		if (self->pixel_active[xy]) {
			y1 = f[2*xy];
			u = f[2*xy+1];
			y2 = f[2*xy+2];
			v = f[2*xy+3];
			self->segmented[xy] = self->colors_lookup[y1 + (u << 8) + (v << 16)];
			self->segmented[xy+1] = self->colors_lookup[y2 + (u << 8) + (v << 16)];
		}
	}

	SegEncodeRuns(self);
	SegConnectComponents(self);
	SegExtractRegions(self);
	SegSeparateRegions(self);

	// do minimal number of passes sufficient to touch all set bits
	int y = 0;
	while( self->max_area != 0 ) {
		self->max_area >>= CMV_RBITS;
		y++;
	}
	self->passes = y;

	if(my_ioctl(self->fd, VIDIOC_QBUF, &buffer)) {
		return NULL;
	}

	Py_RETURN_NONE;
}

static PyObject *CameraGetBuffer(Camera *self, PyObject *args) {
	//return segmented buffer (usage np.frombuffer(cam.getBuffer(), dtype=np.uint8).reshape(cam.shape()))
	if (!self->started) Py_RETURN_NONE;
	
	/*int size = sizeof(char) * self->width * self->height;
	
	return PyBuffer_FromMemory(self->segmented, size);*/
	
	npy_intp dims[2] = {self->height, self->width};
	PyArrayObject *outArray = (PyArrayObject *) PyArray_SimpleNewFromData(2, dims, NPY_UINT8, self->segmented);
	return PyArray_Return(outArray);
}

static PyObject *CameraGetBlobs(Camera *self, PyObject *args) {
	//get blobs for color, return numpy array [[distance,angle,area,cen_x,cen_y,x1,x2,y1,y2],...]
	int color;
	if (!PyArg_ParseTuple(args, "i", &color)) {
		return NULL;
	}
	
	region *list = SegSortRegions(self->colors[color].list, self->passes);
	int rows = self->colors[color].num;
	int cols = 9;
	int i;
	int n = 0;
	int w = self->width;
	int xy;
	unsigned short cen_x, cen_y;
	unsigned short *pout = (unsigned short *) malloc(rows * cols * sizeof(unsigned short));

	for (i=0; i<rows; i++) {
		cen_x = (unsigned short)round(list[i].cen_x);
		cen_y = (unsigned short)round(list[i].cen_y);
		xy = cen_y * w + cen_x;
		
		pout[n++] = self->loc_r[xy];
		pout[n++] = self->loc_phi[xy];
		pout[n++] = (unsigned short)min(65535 , list[i].area);
		pout[n++] = cen_x;
		pout[n++] = cen_y;
		pout[n++] = (unsigned short)list[i].x1;
		pout[n++] = (unsigned short)list[i].x2;
		pout[n++] = (unsigned short)list[i].y1;
		pout[n++] = (unsigned short)list[i].y2;
	}
	
	npy_intp dims[2] = {rows, cols};
	PyArrayObject *outArray = (PyArrayObject *) PyArray_SimpleNewFromData(2, dims, NPY_UINT16, pout);
	PyArray_ENABLEFLAGS(outArray, NPY_ARRAY_OWNDATA);
	return PyArray_Return(outArray);
}

static PyMethodDef Camera_methods[] = {
	{"set", (PyCFunction)Camera_set_params, METH_VARARGS|METH_KEYWORDS,
		"set_params(key=value, ...)\n\n"
		"Set V4L2 settings"},
	{"get", (PyCFunction)Camera_get_params, METH_VARARGS,
		"get_params(string) -> int\n\n"
		"Get V4L2 param value"},
	{"start", (PyCFunction)Camera_start, METH_NOARGS,
		"start()\n\n"
		"Start video capture."},
	{"stop", (PyCFunction)Camera_stop, METH_NOARGS,
		"stop()\n\n"
		"Stop video capture."},
	{"opened", (PyCFunction)CameraOpened, METH_NOARGS,
		"opened()\n\n"
		"True if camera is opened."},
	{"started", (PyCFunction)CameraStarted, METH_NOARGS,
		"started()\n\n"
		"True if camera is started."},
	{"image", (PyCFunction)Camera_read, METH_NOARGS,
		"image()\n\n"
		"Capture image."},
	{"shape", (PyCFunction)CameraShape, METH_NOARGS,
		"shape()\n\n"
		"Retrieve image dimensions (height, width)."},
	{"setColorMinArea", (PyCFunction)CameraSetColorMinArea, METH_VARARGS,
		"setColorMinArea(int color_id, int min_area)\n\n"
		"Find only blobs larger than min_area"},
	{"setColors", (PyCFunction)CameraSetColors, METH_VARARGS,
		"setColors(nparr)\n\n"
		"Set color lookup table."},
	{"setPixels", (PyCFunction)CameraSetActivePixels, METH_VARARGS,
		"setPixels(nparr)\n\n"
		"Set active pixels table."},
	{"setLocations", (PyCFunction)CameraSetLocations, METH_VARARGS,
		"setLocations(nparr distances, nparr angles)\n\n"
		"Set location lookup table."},
	{"analyse", (PyCFunction)CameraAnalyse, METH_NOARGS,
		"analyse()\n\n"
		"Threshold, find connected components."},
	{"getBuffer", (PyCFunction)CameraGetBuffer, METH_NOARGS,
		"getBuffer()\n\n"
		"Retrieve segmentation buffer."},
	{"getBlobs", (PyCFunction)CameraGetBlobs, METH_VARARGS,
		"getBlobs(int color_id)\n\n"
		"Return connected components with color_id."},
	{NULL}
};

static PyTypeObject Camera_type = {
#if PY_MAJOR_VERSION < 3
	PyObject_HEAD_INIT(NULL) 0,
#else
	PyVarObject_HEAD_INIT(NULL, 0)
#endif
	"pyCMVision.Camera", sizeof(Camera), 0,
	(destructor)Camera_dealloc, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, Py_TPFLAGS_DEFAULT, "Camera(path)\n\nOpens the video device at "
	"the given path and returns an object that can capture images. The "
	"constructor and all methods except close may raise IOError.", 0, 0, 0,
	0, 0, 0, Camera_methods, 0, 0, 0, 0, 0, 0, 0,
	(initproc)Camera_init
};

static PyMethodDef module_methods[] = {
	{NULL}
};

#if PY_MAJOR_VERSION < 3
PyMODINIT_FUNC initpyCMVision(void)
#else
PyMODINIT_FUNC PyInit_pyCMVision(void)
#endif
{
	Camera_type.tp_new = PyType_GenericNew;

	if(PyType_Ready(&Camera_type) < 0) {
#if PY_MAJOR_VERSION < 3
		return;
#else
		return NULL;
#endif
	}

	PyObject *module;

#if PY_MAJOR_VERSION < 3
	module = Py_InitModule3("pyCMVision", module_methods,
			"Capture video with video4linux2.");
#else
	static struct PyModuleDef moduledef = {
		PyModuleDef_HEAD_INIT,
		"pyCMVision",
		"Capture video with video4linux2.",
		-1,
		module_methods,
		NULL,
		NULL,
		NULL,
		NULL
	};
	module = PyModule_Create(&moduledef);
#endif

	if(!module) {
#if PY_MAJOR_VERSION < 3
		return;
#else
		return NULL;
#endif
	}

	Py_INCREF(&Camera_type);
	PyModule_AddObject(module, "Camera", (PyObject *)&Camera_type);
	import_array();
#if PY_MAJOR_VERSION >= 3
	return module;
#endif
}
