#gear calculator 2025 Oct 13 (by @dabl2928 on youtube)
import math

t=input("Gear type (internal/external): ")
d=float(input("Gear diameter (planning circle diameter in studs): "))
m=float(input("Teeth size (module): "))

if round(d/m)%2==1:
    odd = True
else:
    odd = False

def calculate(t,d,m,o):
    n_teeth = d/m

    if t.lower() == "external":
        inner_d = d-2.4*m
        outer_d = d+2*m
        circle_piece = round(inner_d*math.tan(math.pi/n_teeth),2)
    else:           #INTERNAL
        thickness = float(input("Outside thickness (studs): "))
        inner_d = d+2.4*m
        outer_d = d+2.4*m+2*thickness
        circle_piece = round(outer_d*math.tan(math.pi/n_teeth),2)

    if t.lower()=="internal":
        correction_factor = 0.6 #IMPROVES SMOOTHNESS (smaller tooth height)
        tooth_height = round(correction_factor*2.2*m,2)
    else:
        tooth_height = round(2.2*m,2)

    if 4 <= n_teeth <= 5:
        g = 0.6
    elif 6 <= n_teeth <= 7:
        g = 0.7
    elif 8 <= n_teeth <= 12:
        g = 0.8
    elif 13 <= n_teeth <= 22:
        g = 0.9
    elif 23 <= n_teeth <= 57:
        g= 1
    else:
        g = 1.1

    tooth_arc_length = round(m*g,2)
    deg = round(360/n_teeth,2)

    if t.lower()=="internal":
        AIS0 = round(inner_d/2+1,2)
    else:
        AIS0 = round(inner_d/2-1,2)

    AIS1 = round((circle_piece-2)/2,2)
    AIS2 = round((tooth_arc_length-2)/2,2)
    AIS3 = round(tooth_height-2,2)

    ##############################################
    print("\n\n========GEAR-INFO========")
    print("Type: "+str(t))
    print("Number of teeth: "+str(round(n_teeth)))
    print("Module (teeth size): "+str(m)+"\n")

    if odd==True:
        if t.lower()=="internal":
            print("Inner radius: "+str(round(inner_d/2,2))+" studs  (move for: "+str(AIS0)+")")
            print("Outer diameter: "+str(outer_d)+" studs  ")
            print("Thickness: "+str(thickness)+" studs  (amount in scale: "+str(round(thickness-2,2))+")")
        else:
            print("Inner radius:"+ str(round(inner_d/2),2)+ " studs  (amount in scale: "+str(AIS0)+")")
 
    else: #EVEN
        if t.lower()=="internal":
            print("Inner diameter: "+str(inner_d)+" studs  (move for: "+str(AIS0)+" on both sides)")
            print("Outer diameter: "+str(outer_d)+" studs")
            print("Thickness: "+str(thickness)+" studs  (amount in scale: "+str(round(thickness-2,2))+")")
        else:
            print("Inner diameter: "+str(inner_d)+" studs  (amount in scale: "+str(AIS0)+" for both sides)")

    print("Circle piece: "+str(circle_piece)+" studs  (amount in scale: "+str(AIS1)+")")
    print("Tooth length: "+str(tooth_arc_length)+" studs  (amount in scale: "+str(AIS2)+")")
    print("Tooth height: "+str(tooth_height)+" studs  (amount in scale: "+str(AIS3)+")")

    print("\nDegrees for rotating and cloning: "+str(deg))

    print("\n==========CHECK-DIMENSIONS==========\nTooth part dimensions: "+str(round(2+2*AIS2,2))+" x "+str(tooth_height))
    if odd==True:
        if t.lower()=="internal": #ODD
            print("Other part dimensions: "+str(round(2+2*AIS1,2))+" x "+str(thickness))
        else:          #EXTERNAL  ODD
            print("Other part dimensions: "+str(round(2+2*AIS1,2))+" x "+str(inner_d/2))
    else:
        if t.lower()=="internal": #EVEN
            print("Other part dimensions: "+str(round(2+2*AIS1,2))+" x "+str(thickness))
        else:          #EXTERNAL   EVEN
            print("Other part dimensions: "+str(round(2+2*AIS1,2))+" x "+str(inner_d))

    if odd==True:print("\nWARNING: gear has an odd number of teeth (add another large part for cloning and rotating)")

if d==0:
    print("ERROR: Invalid diameter (Diameter: "+str(d)+")")
    exit()
elif m==0:
    print("ERROR: Invalid module (Module: "+str(m)+")")
    exit()
elif (d/m)<4:
    print("ERROR: Invalid diameter to module ratio (not enough teeth) (Number of teeth: "+str(d/m)+")")
    exit()
elif round(d/m)!=round(d/m,5):
    print("ERROR: Invalid diameter to module ratio (number of teeth isn't a full number) (Number of teeth: "+str(d/m)+")")
    exit()
elif t.lower()!="internal" and t.lower()!="external":
    print("ERROR: Specify if the gear is internal or external")
    exit()
else:
    calculate(t,d,m,odd)