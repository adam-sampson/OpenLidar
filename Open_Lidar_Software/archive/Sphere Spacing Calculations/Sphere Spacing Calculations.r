# Calculate spacing of points on sphere broken down by longitude

# Define Functions
cap_h <- function(theta,sp_radius){
  sp_radius*(1-cos(theta))
}

cap_r <- function(sp_radius,lat_height){
  sqrt(2*sp_radius*lat_height - lat_height^2)
}

deg_to_rad <- function(deg){
  rad <- deg*pi/180
}

arc_length <- function(theta,cir_radius){
  2*pi*cir_radius*(theta/360)
}

arc_to_theta <- function(arc,cir_radius){
  360*arc/(2*pi*cir_radius)
}

# Define Rules
# Motor is a 0.9 deg per step motor with microstepping
myradius <- 10
microsteps <- 4
pitch_spacing <- deg_to_rad(0.9/microsteps)

# The primary axis is the pitch axis. This always has the
# same spacing, and the arc length of these points is the
# metric that will be used to determine the spacing for the 
# yaw axis.

# pitch arc length for a 10m radius sphere
pitch_arc <- arc_length(pitch_spacing,10)

# The yaw spacing should be as close to the pitch spacing 
# as possible without going over.
# Calculate a boolean dataframe indicating which columns to
# perform a measurement at. Using df instead of matrix because
# this is columnar data.
# This is symetrical, so only have to do the top half.
pos_df <- as.data.frame(matrix(FALSE,
                 nrow = round(2*pi/pitch_spacing)/2,
                 ncol = round(2*pi/pitch_spacing)/2))

for(i in seq(0,deg_to_rad(90),by = pitch_spacing)[1:400]){
  lat_notches <- round(2*pi/arc_to_theta(pitch_arc,cap_r(myradius,cap_h(i,myradius))))
  for(j in seq(0,deg_to_rad(360),by = pitch_spacing)[1:1600]){
    
  }
}

