#include <vigir_footstep_planning_msgs/parameter_set.h>

namespace vigir_footstep_planning
{
ParameterSet::ParameterSet(const std::string& name)
  : name(name)
{
}

ParameterSet::ParameterSet(const XmlRpc::XmlRpcValue& val)
{
  fromXmlRpcValue(val);
}

ParameterSet::ParameterSet(const msgs::ParameterSet& params)
{
  fromMsg(params);
}

ParameterSet::~ParameterSet()
{
}

std::ostream& operator<<(std::ostream& os, const ParameterSet& params)
{
  os << "Name: " << params.getName() << ", size: " << params.params.size();

  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = params.params.begin(); itr != params.params.end(); itr++)
    os << "\n" << itr->first << ": " /*<< itr->second*/;

  return os;
}

void ParameterSet::clear()
{
  name = "";
  params.clear();
}

unsigned int ParameterSet::size() const
{
  return params.size();
}

void ParameterSet::setName(const std::string& name)
{
  setParam("name", name);
}

const std::string& ParameterSet::getName() const
{
  return name;
}

template<>
void ParameterSet::setParam(const std::string& key, const XmlRpc::XmlRpcValue &p)
{
  if (p.valid())
  {
    // strip '/' from key
    std::string _key = strip_const(key, '/');

    if (_key.empty())
    {
      ROS_WARN("[setParam] Got empty key. Skipping!");
      return;
    }

    // key should be always lower case
    std::transform(_key.begin(), _key.end(), _key.begin(), ::tolower);

    // special case for 'name' key
    if(_key == "name")
    {
      if (p.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        std::string old_name = name;
        name = static_cast<std::string>(XmlRpc::XmlRpcValue(p));

        if (old_name == name)
          return;

        if (!old_name.empty())
          ROS_INFO("[ParameterSet] Renamed parameter set '%s' to '%s'.", old_name.c_str(), name.c_str());
      }
      else
      {
        ROS_ERROR("[setParam] Parameter 'name' must be a string!");
        return;
      }
    }

    params[_key] = p;
  }
  else
    ROS_ERROR("[setParam] Type of parameter '%s' not supported!", key.c_str());
}

template<>
void ParameterSet::setParam(const std::string& key, const ros::NodeHandle& nh)
{
  try
  {
    if (!nh.hasParam(key))
    {
      ROS_WARN("[setParam] Requested parameter '%s' was not found in '%s'!", key.c_str(), (nh.getNamespace() + "/" + key).c_str());
      return;
    }

    XmlRpc::XmlRpcValue p;
    nh.getParam(key, p);
    setParam(key, p);
  }
  catch(std::exception& e)
  {
    ROS_ERROR("[setParam] Catched exception while retrieving '%s': %s", key.c_str(), e.what());
    return;
  }
}

void ParameterSet::setParam(const msgs::Parameter& msg)
{
  XmlRpc::XmlRpcValue val;
  val << msg.data;
  setParam(msg.key.data, val);
}

template<>
bool ParameterSet::getParam(const std::string& key, XmlRpc::XmlRpcValue& p) const
{
  p = XmlRpc::XmlRpcValue();

  std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = params.find(key);
  if (itr == params.end())
  {
    ROS_ERROR("[getParam] Couldn't find parameter '%s'!", key.c_str());
    return false;
  }

  p = itr->second;
  return true;
}

template<>
bool ParameterSet::getParam(const std::string& key, unsigned int& p) const
{
  return getParam(key, (int&)p);
}

template<>
bool ParameterSet::getParam(const std::string& key, float& p) const
{
  double _p;
  bool success = getParam(key, _p);
  p = _p;
  return success;
}

template<>
bool ParameterSet::getParam(const std::string& key, msgs::Parameter& p) const
{
  XmlRpc::XmlRpcValue val;
  if (!getParam(key, val))
    return false;

  p.key.data = key;
  return p.data << val;
}

bool ParameterSet::hasParam(const std::string& key) const
{
  return params.find(key) != params.end();
}

bool ParameterSet::fromXmlRpcValue(const XmlRpc::XmlRpcValue& val)
{
  XmlRpc::XmlRpcValue& _val = const_cast<XmlRpc::XmlRpcValue&>(val); // needed because XmlRpc doesn't implement const getters

  if (_val.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("[fromXmlRpcValue] Invalid format given! Expected TypeStruct.");
    return false;
  }

  if (!_val.hasMember("name") || _val["name"].getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_ERROR("[fromXmlRpcValue] Invalid format: Parameter set must contain 'name'!");
    return false;
  }

  // start parsing recursively
  return addXmlRpcValue("", _val);
}

void ParameterSet::updateFromMsg(const msgs::ParameterSet& params)
{
  for (std::vector<msgs::Parameter>::const_iterator itr = params.parameters.begin(); itr != params.parameters.end(); itr++)
    setParam(*itr);
  setName(params.name.data);
}

void ParameterSet::fromMsg(const msgs::ParameterSet& params)
{
  clear();
  updateFromMsg(params);
}

void ParameterSet::toMsg(msgs::ParameterSet& params) const
{
  params.parameters.clear();

  params.name.data = name;

  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = this->params.begin(); itr != this->params.end(); itr++)
  {
    msgs::Parameter param;
    param.key.data = itr->first;
    param.data << itr->second;
    params.parameters.push_back(param);
  }
}

std::string ParameterSet::toString() const
{
  std::ostringstream ss;

  ss << "Set name: " << name;
  for (std::map<std::string, XmlRpc::XmlRpcValue>::const_iterator itr = this->params.begin(); itr != this->params.end(); itr++)
    ss << "\n" << itr->first << ": " << vigir_footstep_planning::toString(itr->second);

  return ss.str();
}

bool ParameterSet::addXmlRpcValue(const std::string& ns, XmlRpc::XmlRpcValue& val)
{
  switch (val.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    case XmlRpc::XmlRpcValue::TypeInt:
    case XmlRpc::XmlRpcValue::TypeDouble:
    case XmlRpc::XmlRpcValue::TypeString:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeArray:
    {
      setParam(ns, val);
      return true;
    }
    case XmlRpc::XmlRpcValue::TypeStruct:
    {
      bool result  = true;
      for (XmlRpc::XmlRpcValue::iterator itr = val.begin(); itr != val.end() && result; itr++)
        result = addXmlRpcValue(ns + (ns.empty() ? itr->first : "/" + itr->first), itr->second);
      return result;
    }
    default:
      ROS_ERROR("[addXmlRpcValue] Unknown type '%u'!", val.getType());
      return false;
  }
}
}
