#include "GpsPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(GpsPlugin)

/////////////////////////////////////////////////
GpsPlugin::GpsPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
GpsPlugin::~GpsPlugin()
{ 
}   

/////////////////////////////////////////////////
void GpsPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{   
  // Get the parent sensor.
  this->parentSensor =
    boost::shared_dynamic_cast<sensors::GpsSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
    { 
      gzerr << "GpsPlugin requires a GpsSensor.\n";
      return;
    } 

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
							      boost::bind(&GpsPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void GpsPlugin::OnUpdate()
{
  // math::Angle lon = this->parentSensor->GetLongitude();
  // math::Angle lat = this->parentSensor->GetLatitude();

  // std::cout << "Latitude:" << lat << std::endl;
  // std::cout << "Longitude:" << lon << std::endl;
}
