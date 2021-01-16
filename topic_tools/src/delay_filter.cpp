///////////////////////////////////////////////////////////////////////////////
// relay just passes messages on. it can be useful if you're trying to ensure
/////////////////////////////////////////////////////////////////////////////
#include <cstdio>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <queue>
using namespace std;
using namespace topic_tools;

ros::NodeHandle *g_node = NULL;
bool g_advertised = false;
string g_input_topic;
string g_output_topic;
string g_monitor_topic;
ros::Publisher g_pub;
ros::Subscriber* g_sub;
bool g_lazy;
int delaytime;
bool g_stealth;
bool startpub = false;
ros::TransportHints g_th;
boost::mutex mutex;
std::queue<boost::shared_ptr<ShapeShifter const>> myqueue;
void conn_cb(const ros::SingleSubscriberPublisher&);
void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event);

void subscribe()
{
  g_sub = new ros::Subscriber(g_node->subscribe(g_input_topic, 10, &in_cb, g_th));
}

void unsubscribe()
{
  if (g_sub)
  {
    delete g_sub;
    g_sub = NULL;
  }
}

void conn_cb(const ros::SingleSubscriberPublisher&)
{
  // If we're in lazy subscribe mode, and the first subscriber just
  // connected, then subscribe, #3389.
  if(g_lazy && !g_stealth && !g_sub)
  {
    ROS_DEBUG("lazy mode; resubscribing");
    subscribe();
  }
}

void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event)
{
  // boost::mutex::scoped_lock lock(mutex);
  const boost::shared_ptr<ShapeShifter const> msg = msg_event.getConstMessage();
  //myqueue.push(msg);
  boost::shared_ptr<const ros::M_string> const& connection_header = msg_event.getConnectionHeaderPtr();

  if (!g_advertised)
  {
    // If the input topic is latched, make the output topic latched, #3385.
    bool latch = false;
    if (connection_header)
    {
      ros::M_string::const_iterator it = connection_header->find("latching");
      if((it != connection_header->end()) && (it->second == "1"))
      {
        ROS_DEBUG("input topic is latched; latching output topic to match");
        latch = true;
      }
    }
    g_pub = msg->advertise(*g_node, g_output_topic, 10, latch, conn_cb);
    g_advertised = true;
    ROS_INFO("advertised as %s\n", g_output_topic.c_str());
  }

  usleep(8000000);
  // If we're in lazy subscribe mode, and nobody's listening,
  // then unsubscribe, #3389.
  if((g_lazy || g_stealth) && !g_pub.getNumSubscribers())
  {
    ROS_DEBUG("lazy mode; unsubscribing");
    unsubscribe();
  }
  else
  {
    g_pub.publish(msg);
  }
}

void timer_cb(const ros::TimerEvent&)
{
  if (!g_advertised) return;

  // get subscriber num of ~monitor_topic
  XmlRpc::XmlRpcValue req(ros::this_node::getName()), res, data;
  if (!ros::master::execute("getSystemState", req, res, data, false))
  {
    ROS_ERROR("Failed to communicate with rosmaster");
    return;
  }

  int subscriber_num = 0;
  XmlRpc::XmlRpcValue sub_info = data[1];
  for (int i = 0; i < sub_info.size(); ++i)
  {
    string topic_name = sub_info[i][0];
    if (topic_name != g_monitor_topic) continue;
    XmlRpc::XmlRpcValue& subscribers = sub_info[i][1];
    for (int j = 0; j < subscribers.size(); ++j)
    {
      if (subscribers[j] != ros::this_node::getName()) ++subscriber_num;
    }
    break;
  }

  // if no node subscribes to ~monitor, do unsubscribe
  if (g_sub && subscriber_num == 0) unsubscribe();
  // if any other nodes subscribe ~monitor, do subscribe
  else if (!g_sub && subscriber_num > 0) subscribe();
}
// void Work()
// {
//   usleep(8000000);//整体延迟2ms
//   ROS_INFO("start pub!");
//   startpub = true;
//   // simulated worker thread
//   // while(true)
//   // {
//   //   // lock to get data from AtlasState
//   //   {
//   //     //boost::mutex::scoped_lock lock(mutex);
//   //   }
//   //   if(!myqueue.empty())
//   //   {
//   //     const boost::shared_ptr<ShapeShifter const> tem = myqueue.front();
//   //     myqueue.pop();
//   //     g_pub.publish(tem);
//   //   }
//     // simulate working
//     //g_pub.publish(msg);
//   //}
// }
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("\nusage: relay IN_TOPIC [OUT_TOPIC]\n\n");
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(string(argv[1]), topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_relay"),
            ros::init_options::AnonymousName);
  if (argc == 2)
    g_output_topic = string(argv[1]) + string("_relay");
  else // argc == 3
    g_output_topic = string(argv[2]);
  g_input_topic = string(argv[1]);
  ros::NodeHandle n;
  g_node = &n;

  ros::NodeHandle pnh("~");
  bool unreliable = false;
  pnh.getParam("unreliable", unreliable);
  pnh.getParam("lazy", g_lazy);
  pnh.getParam("delay", delaytime);
  if (unreliable)
    g_th.unreliable().reliable(); // Prefers unreliable, but will accept reliable.

  pnh.param<bool>("stealth", g_stealth, false);
  ros::Timer monitor_timer;
  if (g_stealth)
  {
    double monitor_rate;
    pnh.param<string>("monitor_topic", g_monitor_topic, g_input_topic);
    pnh.param<double>("monitor_rate", monitor_rate, 1.0);
    monitor_timer = n.createTimer(ros::Duration(monitor_rate), &timer_cb);
  }

  subscribe();

  //startpub = true;
  //boost::thread work = boost::thread(&Work);
  ros::spin();
  return 0;
}