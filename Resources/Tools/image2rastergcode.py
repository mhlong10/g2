#!/usr/bin/env python

import sys, os
import argparse
import math

from PIL import Image
from PIL import ImageOps

mmpi = 25.4

class TransformMatrix(object):
    def __init__(self):
        self.clear()
        pass
    
    def clear(self):
        # Set with no parameters clears transform
        # i.e. no scale, translate, or rotate
        self.set()
        
    def get(self):
        return (ta,tb,tc,td,te,tf)
    
    def set(self, a=1.0,b=0,c=0,d=1.0,e=0,f=0):
        self.ta = a
        self.tb = b
        self.tc = c
        self.td = d
        self.te = e
        self.tf = f
        
    # matrix is [xp = [a c e * [x
    #            yp    b d f    y
    #            1     0 0 1]   1]
    def transform(self, x, y):
        xp = x*self.ta+y*self.tc+self.te
        yp = x*self.tb+y*self.td+self.tf
        return (xp, yp)
    
    def rotate(self, deg):
        th = deg * math.pi / 180
        self.ta=math.cos(th)
        self.tc=math.sin(th)
        self.tb=-math.sin(th)
        self.td=math.cos(th)
        
    def translate(self, x, y):
        self.te = x;
        self.tf = y;
       
transform1 = TransformMatrix()
transform2 = TransformMatrix()

def dist(v1, v2):
    dx = v2.x-v1.x
    dy = v2.y-v1.y
    return math.sqrt(dx*dx+dy*dy)

    
header = """T1
G21
G90
G0W-0.015
T32
M6
S0
M4
G0Z6.350
G0X0.000Y0.000
"""

footer = """S0
M5
G0Z6.350
G0X0.000Y0.000
"""

w = 0 # W starts at zero because set to zero in header
def getW(nbytes):
    global w
    # Return a W value to step thru data array nbytes (number steps)
    # Note - W axis motor setup for 0.01 mm per step
    if w <= 0.0: w += (nbytes*0.01)
    else: w -= (nbytes*0.01)
    w = round(w * 100.0)/100.0
    return w
    
def isEven(val):
    return (val & 1 == 0)

mlc = 480       # Max Ascii characters per gcode protocol line
mbpl = mlc / 2  # Max bytes per ascii line.  For hex 2 chars = 1 byte so mlc/2
mbps = mbpl * 1 # Max bytes per gcode move segment.  Doesn't have to be even multiple of mbpl
                # Setting this to 1 for now since there is no flow control and mbpl is fairly
                # large size.  Setting higher we quickly overfill the buffer since the gcode
                # moves from the planner fall behind the M100 async fills.

def main(args, original_img):
    cols = int(round(args.width * args.dpmm))
    rows = int(round(args.height * args.dpmm))
    nsegs = int(cols/mbps) + 1 # Integer number of segments
    bpseg = cols/nsegs
    
    fd = None
    imgrs = None
    
    def getPower(x, y):
        int_power = (imgrs.getpixel((x, y)) >> (8 - args.bits)) << (8 - args.bits)
        return (args.power_min + (((255-int_power) / 255.0) * (args.power_max - args.power_min)))
    
    def gcode_emit(gcode):
#    print gcode,
        fd.write(gcode)

    def gcode_emit_line(line):
        gcode_emit((line + '\n'))
    
    img = ImageOps.autocontrast(ImageOps.grayscale(original_img), cutoff=args.cutoff)
    imgrs = img.resize((cols, rows), Image.BICUBIC)

    if (args.preview):
        imgrs.show()
        return 0

#    imgrs.show()
#    imgrs.save("scaled_image.jpg")

#    print ('Image pixels - Width px: {} Height px: {}'.format(imgrs.width, imgrs.height))
#    print ('Image pixels - Width mm(inch): {:.3f}({:.3f}) Height mm/inch: {:.3f}({:.3f})'.format(
#        imgrs.width * x_step, imgrs.height * y_step,
#        imgrs.width * (x_step/25.4), imgrs.height * (y_step/25.4)))

    transform1.translate(args.trax, args.tray) # shift image from bottom left prior to rotation
    transform2.rotate(args.rot)                # rotate image around 0, 0 of shifted image
    
    def trans(x, y):
        tx, ty = transform1.transform(x, y)
        tx, ty = transform2.transform(tx, ty)
        return tx, ty
    
    fd = open(args.output_name, 'w')

    gcode_emit(header.format(args.gcode_for_power, args.gcode_for_power))
    
    mx, my = trans(0, args.height)
    gcode_emit_line('G0X{:.3f}Y{:.3f}'.format(mx, my)) # Move to start position power already 0 from G10 in header
    gcode_emit_line('G0Z0') # Move to start height
    gcode_emit_line('G1F{:.3f}'.format(args.feed)) # Set feed rate
    gcode_emit_line('S{:.3f}'.format(getPower(0, 0)*100)) # Start laser at power for first point

    x = 0
    for y in range(0, rows):
        mx, my = trans(args.step_over * x, args.height - (args.step_over * y))
        gcode_emit_line('G1X{:.3f}Y{:.3f}'.format(
            mx, my)) # Move to start position
        b_remain = cols
        tx = 0
        while (b_remain > 0):
            bytes_this_segment = min(bpseg, b_remain)
            h = ""
            for b in range(0, bytes_this_segment):
                # Output Hex data
                power = getPower(x, y) * 255.0 # Fixed 0-255 for now
                h += "{:02x}".format(int(round(power)))
                if (isEven(y)): x = tx;
                else: x = (cols - 1) - tx;
                if b % mbpl == mbpl - 1:
                    gcode_emit_line('M100.1({{pad:"{}"}})'.format(h))
                    h = ""
                tx += 1
            gcode_emit_line('M100.1({{pad:"{}"}})'.format(h))
            mx, my = trans(args.step_over * x, args.height - (args.step_over * y))
            gcode_emit_line('G1X{:.3f}Y{:.3f}W{}'.format(mx, my, getW(bytes_this_segment)-0.005))
            b_remain -= bytes_this_segment
               
    
    gcode_emit(footer)    
    
    fd.close()

def normalize_float_to_mm(s):
    if s.endswith("in"): r = float(s[:-2]) * mmpi
    elif s.endswith("mm"): r = float(s[:-2])
    else: r = float(s)
    return r

def normalize_dpi_to_dpmm(s):
    if s.endswith('dpmm') or s.endswith('dpin'): s = s[:-4] + s[-2:]
    elif s.endswith('dpi'): s = s[:-3] + 'in'
    if s.endswith("in"): r = float(s[:-2]) / mmpi
    elif s.endswith("mm"): r = float(s[:-2])
    else: r = float(s) / mmpi
    return r

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Image to gcode")
    parser.add_argument("image_name", type=str,
                        help="File name of image to convert")
    parser.add_argument("-o", "--output_name", type=str, default="",
                        help="File name of gcode output file.  If not provided then default to '<image-name>.gcode'.")
    parser.add_argument("-d", "--dpi", type=str, default="100",
                        help="Dots per inch or dots per mm if suffixed with 'dpmm' e.g. '10.23dpmm'.  If not provided then default to 100dpi.")
    parser.add_argument("-w", "--width", type=str, default="",
                        help="""Width of printed image in mm or inches if suffixed with 'in' e.g. '10.0in'.  If neither width or height are specified 
                        then image isn't scaled and dpi setting determines size e.g. 100dpi * 200x150 image size -> 2in x 1.5in output size. If width
                        or height are specified then the scale factor will be used for both e.g. Using previous example, if width of 4in is specified
                        then output will be 4in x 3in.  If both width and height are specified then each dimension will be scaled to meet dimensions.""")
    parser.add_argument("-t", "--height", type=str, default="",
                        help="Height of printed image in mm or inches if suffixed with 'in' e.g. '10.0in'. See 'Width' for more details.")
    parser.add_argument("-f", "--feed", type=str, default="1800",
                        help="Feed rate in mm/min or in/min if suffixed with 'in'.  If not provided default is 1800mm/min")
    parser.add_argument("-n", "--power_min", type=float, default=0.0,
                        help="Minimum %% laser power.  e.g. 50.0%% = fifty percent power.  If not provided default is 0.0")
    parser.add_argument("-x", "--power_max", type=float, default=50.0,
                        help="Maximum %% laser power.  e.g. 50.0%% = fifty percent power.  If not provided default is 50.0")
    parser.add_argument("-b", "--bits", type=int, default=8, choices=xrange(1,8),
                        help="""Number of bits in grayscale level.  Maximum is 8 minimum is 1.  Default is 8.  Less bit can lead to significantly
                        reduced runtime but at the expense of more course grayscale.""")
    parser.add_argument("-c", "--cutoff", type=float, default=0.0,
                        help="""When normalizing contrast, this is the percentage of darkest and lightest areas which are clipped.  Default is 0.0.""")
    parser.add_argument("--gcode_for_power", type=str, default='W', choices=['W','Z','S'],
                        help="Default is 'W'.  Use 'Z' to visualize power as depth.")
    parser.add_argument("--optimize", type=int, default=1, choices=xrange(0,1),
                        help="Gcode optimization level.  0 == no optimizaion (gcode output for every dot).  Default is 1.")
    parser.add_argument("--preview", action='store_true',
                        help="Preview the image and then exit without generating gcode.")
    parser.add_argument("--trax", type=float, default=0.0,
                        help="""Translate x prior to rotate.  Default is 0.0.""")
    parser.add_argument("--tray", type=float, default=0.0,
                        help="""Translate y prior to rotate.  Default is 0.0.""")
    parser.add_argument("--rot", type=float, default=0.0,
                        help="""Rotate image about it's post translated origin.  Default is 0.0 degrees.""")
    
    args = parser.parse_args()
#    print args.__dict__

    # Process image and gocde output file names
    if not args.output_name: 
        args.output_name = args.image_name + '.gcode'
    f, e = os.path.splitext(args.output_name)
    if not e:
        args.output_name += '.gcode' # Add default extension if it doesn't have one

    # Open image file now so info can be gathered on image
    img = Image.open(args.image_name)

    # Process DPI (default) or DPMM (if specified with 'dpmm' sufix e.g. '10dpmm')
    args.dpmm = normalize_dpi_to_dpmm(args.dpi)

    # For now step_over is = dpmm
    args.step_over = 1.0 / args.dpmm

    # Process width, height
    if args.width:
        args.width = normalize_float_to_mm(args.width)
    if args.height:
        args.height = normalize_float_to_mm(args.height)
    if not args.width and not args.height:
        args.width = img.width / args.dpmm
        args.height = img.height / args.dpmm
    elif not args.width:
        args.width = float(img.width) / float(img.height) * args.height;
    elif not args.height:
        args.height = float(img.height) / float(img.width) * args.width;

    # Process feed rate
    args.feed = normalize_float_to_mm(args.feed)

    # Process laser power min and max
    args.power_min = args.power_min / 100.0
    if args.power_min < 0.0: args.power_min = 0.0
    elif args.power_min > 1.0: args.power_max = 1.0
    args.power_max = args.power_max / 100.0
    if args.power_max < 0.0: args.power_max = 0.0
    elif args.power_max > 1.0: args.power_max = 1.0

    # Print some settings
    print ("\n       Image file name: {}\nGcode output file name: {}\n".format(
        args.image_name, args.output_name))
    print ("Original image size: Width: {}px Height: {}px\n".format(
        img.width, img.height))
    print ("Output:")
    print ("   Size mm(inchs): Width: {:.3f}({:.3f}) Height: {}({})".format(
        args.width, args.width/mmpi, args.height, args.height/mmpi))
    print ("   DPMM(DPI): {:.3f}({:.1f})".format(args.dpmm, args.dpmm*mmpi))
    print ("   Stepover mm(inch): {:.3f}({:.3f})".format(
        args.step_over, args.step_over/mmpi))
    print ("   Resolution: Width: {} dots Height: {} dots".format(
        int(round(args.width * args.dpmm)), int(round(args.height * args.dpmm))))
    print ("   Speed mm/min(inch/min): {:.1f}({:.2f})".format(
        args.feed, args.feed / mmpi))
    print ("   Laser power: Maximum: {:.1f}% Minimum: {:.1f}%".format(
        args.power_max * 100, args.power_min * 100))
    print ("   Grayscale levels(bits): {} ({})".format(pow(2,args.bits), args.bits))

    print ("\nEstimated ideal time: {:.2f} minutes".format((args.width/args.feed)*int(round(args.height * args.dpmm))))

    sys.exit( main(args, img) )