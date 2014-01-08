
bool checkIntersection(Waypoint& p1, Waypoint& p2, Waypoint& pp1, Waypoint& pp2,
			    Waypoint a, Waypoint b, Waypoint c, double radius){

  // VARIABLE DEFINITIONS
  // a : 1st point in line
  // b : 2nd point in line
  // c : approximate location of pole
  // pp1 : entrance point 
  // pp2 : exit point
  // pp3 : side point (thin)
  // pp4 : side point (fat)
  // *all points are in ENU

  double line_dist = sqrt((b.x-a.x)*(b.x-a.x) + (b.y-a.y)*(b.y-a.y));
  double Dirx = (b.x-a.x)/line_dist; //direction vector x
  double Diry = (b.y-a.y)/line_dist; //direction vector y
  double t = Dirx*(c.x-a.x) + Diry*(c.y-a.y);
  
  //now line eq. is x = Dirx*t + a.x, y = Diry*t + a.y , 0 <= t <= 1

  //find coordinates of projected point on line
  Waypoint e;
  e.x = t * Dirx + a.x;
  e.y = t * Diry + a.y;
  
  //find distance from proj point to center of circle
  ec_dist = sqrt( (e.x-c.x)*(e.x-c.x) + (e.y-c.y)*(e.y-c.y) );

  if(ec_dist < radius){

    // line intersects with circle
    // find distance from proj point to center of circle
    dt = sqrt( radius*radius - ec_dist*ec_dist );
    
    Waypoint temp1, temp2;
    temp1.x = (t-dt)*Dirx + a.x; 
    temp1.y = (t-dt)*Diry + a.y; 
    temp2.x = (t+dt)*Dirx + a.x; 
    temp2.y = (t+dt)*Diry + a.y; 
    
    // check distances to make sure entrance/exit points labeled correctly
    dist_temp1 = sqrt( (temp.x-a.x)*(temp.x-a.x) + (temp.y-a.y)*(temp.y-a.y) );
    if(dist_temp1 < dist_center){
      p1.x = temp1.x;
      p1.y = temp1.y;
      p2.x = temp2.x;
      p2.y = temp2.y;
    }
    else{
      p1.x = temp2.x;
      p1.y = temp2.y;
      p2.x = temp1.x;
      p2.y = temp1.y;
    }

    // this is a magic simplification of what was just done to find the
    // first two intersection points but now we're using the projected point
    // and the center of the circle to get a perpendicular line to find the
    // side waypoints... and I just sort of simplified it down a bit. 

    double perp_line_dist = sqrt((c.x-e.x)*(c.x-e.x) + (c.y-e.y)*(c.y-e.y));
    double perp_Dirx = (c.x-e.x)/perp_line_dist; 
    double perp_Diry = (c.y-e.y)/perp_line_dist;
    
    Waypoint tempside1, tempside2;

    tempside1.x = (perp_line_dist-radius) * perp_Dirx + e.x;
    tempside1.y = (perp_line_dist-radius) * perp_Diry + e.y;
    tempside2.x = (perp_line_dist+radius) * perp_Dirx + e.x;
    tempside2.y = (perp_line_dist+radius) * perp_Diry + e.y;

    // assign side points based on which one is thin(pp3) / fat(pp4)

    dist_tempside1 = sqrt( (tempside1.x-e.x)*(tempside1.x-e.x) + (tempside1.y-e.y)*(tempside1.y-e.y) );
    if(dist_tempside1 < radius){
      pp3.x = tempside1.x;
      pp3.y = tempside1.y;
      pp4.x = tempside2.x;
      pp4.y = tempside2.y;
    }
    else {
      pp3.x = tempside2.x;
      pp3.y = tempside2.y;
      pp4.x = tempside1.x;
      pp4.y = tempside1.y;
    }

    //return true because pole is in path
    return true;

  }
  else if( ec_dist == radius){ 
    // line is tangent to circle. let's not do anything about it for now.
    // I'm just hoping this never happens.
    return false;
  }
  else {
    // line doesn't intersect circle, ain't nothin' doin'.
    return false;
  }

}
