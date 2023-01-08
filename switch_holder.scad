
$fn=50;

inches = 25.4;

depth = 14;  // mm
width = 0.625*inches;
thickness = 4.0;  // Wall thickness
flare = 0.375 * inches;  // Additional radius of top rim


module cavity() {
    hull() {
        translate([0,-0.255*inches,0]) cylinder(depth,d=width);
        translate([0,0.255*inches,0]) cylinder(depth,d=width);
    }
}


module bowl() {
    translate([0,0,-depth]) {
        difference() {
            minkowski() {
                cavity();
                translate([0,0,-thickness/2]) cylinder(thickness,r=thickness,center=true);
            }
            cavity();
            translate([0,0,2*thickness]) cavity();
        }
    }  
}
    
    
module rim() {
    translate([0,0,-thickness/2]) {
        intersection() {
            cube([3*inches, 3*inches, thickness], center=true);
            translate([0,0,-5]) {
                difference() {
                    minkowski() {
                        cavity();
                        cylinder(thickness,r=flare,center=true);
                    }
                    cavity();
                    translate([0,0,20]) cavity();
                }
            }
        }
    }
}

hole_y = 24;
hole_z = (depth+thickness)/2;

module bracket() {
    height = 2*(hole_y+6);
    translate([0,-height/2,-(2*hole_z)]) cube([5, height, 2*hole_z]);
    translate([0,-10,-(2*hole_z)]) cube([8, 20, 2*hole_z]);
}

difference() {
    union() {
        translate([(width/2)+flare,0,0]) {
            bowl();
            rim();
        }
        bracket();
    }

    translate([(width/2)+flare,0,-depth]) cylinder(40, d=6.32, center=true);
    #translate([0,hole_y,-hole_z]) rotate([0,90,0]) cylinder(20, d=4.0, center=true);
    #translate([0,-hole_y,-hole_z]) rotate([0,90,0]) cylinder(20, d=4.0, center=true);

}

