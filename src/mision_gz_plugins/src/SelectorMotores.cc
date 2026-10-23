// Sistema de Gazebo que decide quién mueve los motores del x500 cuando lo pilota
// ArduPilot SITL: ArduPilot (un gz.msgs.Double por motor, como los publica
// ArduPilotPlugin con <type>COMMAND</type>) o la red neuronal del proyecto
// (un gz.msgs.Actuators con las velocidades de rotor, de controlador_red).
// El resultado va a MulticopterMotorModel, así que la física de los motores es
// la misma con los dos controladores.
//
// La fuente se elige publicando "ardupilot" o "red" (gz.msgs.StringMsg) en el
// tópico <modo>. Si la red deja de publicar más de <tiempo_max> segundos
// (tiempo de simulación) se regresa sola a ArduPilot.
//
// Uso en el SDF del modelo:
//   <plugin filename="SelectorMotores" name="mision::SelectorMotores">
//     <entrada>/x500/ap/motor_0</entrada>   (una por motor, en orden)
//     ...
//     <red>/x500/red/motor_speed</red>
//     <modo>/x500/controlador</modo>
//     <salida>/x500_busqueda_0/command/motor_speed</salida>   (donde escucha MulticopterMotorModel)
//     <tiempo_max>0.05</tiempo_max>
//   </plugin>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace mision
{
class SelectorMotores : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    auto elem = std::const_pointer_cast<sdf::Element>(_sdf);
    for (auto e = elem->FindElement("entrada"); e; e = e->GetNextElement("entrada"))
    {
      const size_t i = this->ap.size();
      this->ap.push_back(0.0);
      std::function<void(const gz::msgs::Double &)> cb = [this, i](const gz::msgs::Double &_m)
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->ap[i] = _m.data();
        this->nuevoAp = true;
      };
      this->nodo.Subscribe(e->Get<std::string>(), cb);
    }
    this->red.assign(this->ap.size(), 0.0);
    std::function<void(const gz::msgs::Actuators &)> cbRed = [this](const gz::msgs::Actuators &_m)
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      for (int i = 0; i < _m.velocity_size() && i < static_cast<int>(this->red.size()); ++i)
        this->red[i] = _m.velocity(i);
      this->recibidaRed = true;
    };
    this->nodo.Subscribe(_sdf->Get<std::string>("red", "/x500/red/motor_speed").first, cbRed);
    std::function<void(const gz::msgs::StringMsg &)> cbModo = [this](const gz::msgs::StringMsg &_m)
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->modoRed = (_m.data() == "red");
      gzmsg << "SelectorMotores: motores controlados por " << (this->modoRed ? "la red" : "ArduPilot") << "\n";
    };
    this->nodo.Subscribe(_sdf->Get<std::string>("modo", "/x500/controlador").first, cbModo);
    this->tiempoMax = _sdf->Get<double>("tiempo_max", 0.05).first;
    const auto salida = _sdf->Get<std::string>("salida", "command/motor_speed").first;
    this->pub = this->nodo.Advertise<gz::msgs::Actuators>(salida);
    gzmsg << "SelectorMotores: " << this->ap.size() << " motores de ArduPilot o la red -> [" << salida << "]\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &) override
  {
    if (_info.paused)
      return;
    const double t = std::chrono::duration<double>(_info.simTime).count();
    gz::msgs::Actuators msg;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->recibidaRed)
      {
        this->tRed = t;
        this->recibidaRed = false;
        this->nuevoRed = true;
      }
      if (this->modoRed && t - this->tRed > this->tiempoMax)
      {
        gzwarn << "SelectorMotores: la red dejó de publicar; regreso a ArduPilot\n";
        this->modoRed = false;
      }
      const bool usarRed = this->modoRed;
      if (usarRed ? !this->nuevoRed : !this->nuevoAp)
        return;
      (usarRed ? this->nuevoRed : this->nuevoAp) = false;
      for (double w : (usarRed ? this->red : this->ap))
        msg.add_velocity(w);
    }
    this->pub.Publish(msg);
  }

private:
  gz::transport::Node nodo;
  gz::transport::Node::Publisher pub;
  std::mutex mutex;
  std::vector<double> ap, red;
  bool nuevoAp{false}, nuevoRed{false}, recibidaRed{false}, modoRed{false};
  double tRed{-1.0}, tiempoMax{0.05};
};
}  // namespace mision

GZ_ADD_PLUGIN(mision::SelectorMotores, gz::sim::System,
              mision::SelectorMotores::ISystemConfigure,
              mision::SelectorMotores::ISystemPreUpdate)
