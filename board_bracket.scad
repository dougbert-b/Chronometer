$fn = 30;

board_x = 44.45;
board_y = 64.0;

post_r = 3.0;

module board_post()
{
   cylinder(h=5.0, d=2*post_r);
}

module board_post_hole()
{
   translate([0,0,-6]) cylinder(h=15, d=3.0);   
}

module board_posts()
{
    translate([-board_x/2,-board_y/2]) { board_post(); }
    translate([-board_x/2, board_y/2]) { board_post(); }
    translate([ board_x/2,-board_y/2]) { board_post(); }
    translate([ board_x/2, board_y/2]) { board_post(); }
}

module board_post_holes()
{
    translate([-board_x/2,-board_y/2]) { board_post_hole(); }
    translate([-board_x/2, board_y/2]) { board_post_hole(); }
    translate([ board_x/2,-board_y/2]) { board_post_hole(); }
    translate([ board_x/2, board_y/2]) { board_post_hole(); }
}

module triangle() {
    translate([-(board_x/2+post_r+2+2),-2,-4])
        rotate([90,-90,180])
            linear_extrude(4)
                polygon(points=[[0,0],[0,53],[-15,0]]);
}


difference() {
    union() {
        board_posts();
        
        translate([0,0,-2]) {
            difference() {
                cube([board_x+(2*post_r), board_y+(2*post_r), 4.0], center=true);
                cube([board_x-(3*post_r), board_y-(2*post_r+6), 4.0], center=true);
            }
        }
        
        #translate([-(board_x/2+post_r+1.5),0,-2]) cube([4, board_y+(2*post_r), 4.0], center=true);
        translate([-(board_x/2+post_r+5),0,-6]) cube([4, board_y+(2*post_r), 40.0], center=true);
        
        translate([0,board_y/2-4,0]) triangle();
        translate([0,-(board_y/2-4),0]) triangle();

    }
    #board_post_holes();
}


