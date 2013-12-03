#include "GpsPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

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
    boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
    { 
      gzerr << "GpsPlugin requires a GpsSensor.\n";
      return;
    } 

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
							      boost::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void GpsPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
    {
      std::cout << "Collision between[" << contacts.contact(i).collision1()
		<< "] and [" << contacts.contact(i).collision2() << "]\n";

      for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
	{
	  std::cout << j << "  Position:"
		    << contacts.contact(i).position(j).x() << " "
		    << contacts.contact(i).position(j).y() << " "
		    << contacts.contact(i).position(j).z() << "\n";
	  std::cout << "   Normal:"
		    << contacts.contact(i).normal(j).x() << " "
		    << contacts.contact(i).normal(j).y() << " "
		    << contacts.contact(i).normal(j).z() << "\n";
	  std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
	}
    }
  
  // math::Angle lon = this->parentSensor->GetLongitude();
  // math::Angle lat = this->parentSensor->GetLatitude();

  // std::cout << "Latitude:" << lat << std::endl;
  // std::cout << "Longitude:" << lon << std::endl;
}
