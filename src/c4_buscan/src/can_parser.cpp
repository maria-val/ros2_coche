#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "buscan_msgs/msg/can_raw.hpp"
#include "buscan_msgs/msg/can_msg.hpp"
#include "c4_buscan/buscan.hpp"

class CanParser : public rclcpp::Node
{
  public:
    CanParser() : Node("can_parser")
    {
      subscription_ = this->create_subscription<buscan_msgs::msg::CanRaw>("buscan", 100, std::bind(&CanParser::parse_can_msg, this, std::placeholders::_1));
      publisher_ = this->create_publisher<buscan_msgs::msg::CanMsg>("can_parser", 100);
    }

  private:
    void parse_can_msg(const buscan_msgs::msg::CanRaw msgIn)
    {
      buscan_msgs::msg::CanMsg msgOut;
      RCLCPP_INFO(this->get_logger(), "Mensaje recibido");

      BUSCAN_crude_data_t crude_buscan_data;

      parse_buscan_id(&crude_buscan_data, msgIn.id, msgIn.time, &(msgIn.raw[0]));

      if(msgIn.id == 781)
      {
        msgOut.speed = (crude_buscan_data.data_t3.vrti + crude_buscan_data.data_t3.vrti) / 2.0;
        msgOut.steer = crude_buscan_data.data_t2.volante_angulo;
        msgOut.brake = crude_buscan_data.data_t8.freno_servicio_intensidad;
        msgOut.throttle = crude_buscan_data.data_t0.pedal_acelerador;
        msgOut.gear = crude_buscan_data.data_t15.marchas;
        msgOut.header = msgIn.header;

        rclcpp::Clock time;
        msgOut.header.stamp = time.now();
        publisher_->publish(msgOut);
      }
    }

    void parse_buscan_id(BUSCAN_crude_data_t *crude_buscan_data, uint32_t id, uint32_t time, const unsigned char *msg)
    {
      int freno_aux = 0;
      unsigned char intermitentes_aux = 0;
      
      switch (id)
      {
        case 520: //ID RPM, FRENO y PEDAL ACELERADOR
          crude_buscan_data->data_t0.timestamp = time;
          //RPM
          crude_buscan_data->data_t0.rpm=((double)(msg[0]*256+msg[1])/8);
          //PEDAL ACELERADOR
          crude_buscan_data->data_t0.pedal_acelerador= (double)(msg[3]*0.5);
          break;
        case 761: //ID
          crude_buscan_data->data_t1.timestamp = time;
          break;
        case 773: //ID VOLANTE
          crude_buscan_data->data_t2.timestamp = time;
          //VOLANTE ANGULO
          /*			if (msg[0] > 0x15) //Angulo sentido horario
                      dataBUSCAN->data_t2.volante_angulo=(double)((65535-((msg[0]*256)+msg[1]))*0.1634);  //Grados
                  else  //Angulo sentido antihorario
                      dataBUSCAN->data_t2.volante_angulo=(double)(((msg[0]*256)+msg[1])*-0.1634);  //Grados
          */
          crude_buscan_data->data_t2.volante_angulo=(double)(((msg[0]*256)+msg[1]));
          if (crude_buscan_data->data_t2.volante_angulo>7200.0)
              crude_buscan_data->data_t2.volante_angulo -= 65535.0;
          crude_buscan_data->data_t2.volante_angulo=(-crude_buscan_data->data_t2.volante_angulo*0.1);

          //VOLANTE VELOCIDAD
          crude_buscan_data->data_t2.volante_velocidad = (msg[2]*4); //grados/s
          //VOLANTE SENTIDO
          if (crude_buscan_data->data_t2.volante_velocidad > 0)
          {
            if (msg[3] > 0x7F) //el bit msg[3].7 esta a 1
              crude_buscan_data->data_t2.volante_sentido = 1; //sent horario
            else
              crude_buscan_data->data_t2.volante_sentido = -1; //sent antihorario
          }
          else
            crude_buscan_data->data_t2.volante_sentido = 0; //parado
          break;
        case 781: //ID 4 VELOCIDADES
          crude_buscan_data->data_t3.timestamp = time;
          crude_buscan_data->data_t3.vrdi = ((double)(msg[0]*256+msg[1])*0.01);
          crude_buscan_data->data_t3.vrdd = ((double)(msg[2]*256+msg[3])*0.01);
          crude_buscan_data->data_t3.vrti = ((double)(msg[4]*256+msg[5])*0.01);
          crude_buscan_data->data_t3.vrtd = ((double)(msg[6]*256+msg[7])*0.01);

          break;
        case 840: //ID Algun sensor entrada de aire??
          crude_buscan_data->data_t4.timestamp = time;
          break;
        case 841: //ID MARCHAS
          crude_buscan_data->data_t5.timestamp = time;
          break;
        case 845: //ID ESP Interruptor ON/OFF y Activado si salta
          crude_buscan_data->data_t6.timestamp = time;
          //ESP
          if(msg[0] > 0x7F) //el bit msg[0].7 esta a 1
            crude_buscan_data->data_t6.ESP_activado = 0; // si msg[0]=0x8X: Interruptor ESP ON/OFF = OFF
          else
            crude_buscan_data->data_t6.ESP_activado = 1; // si msg[0]=0x0X: Interruptor ESP ON/OFF = ON
          if(msg[0] == 0x45)
            crude_buscan_data->data_t6.ESP_disparado = 1; // ESP Activado (Ha saltado)
          else
            crude_buscan_data->data_t6.ESP_disparado = 0; // ESP Desactivado
          break;
        case 909: //ID ABS VEHICLE DINAMICS: Vel en delanteras, Distancia traseras y Aceleracion longitudinal
          crude_buscan_data->data_t7.timestamp = time;

          crude_buscan_data->data_t7.vel=((double)(msg[0]*256+msg[1])*0.01); // km/h (0.01)
          crude_buscan_data->data_t7.distancia=((double)(msg[2]*256+msg[3])*0.1); // m (0.1)
          crude_buscan_data->data_t7.acc=((double)(msg[4]*0.08)-14); // m/s2 (0.08)
          break;
        case 973: //ID FRENO ACCION E INTENSIDAD
          crude_buscan_data->data_t8.timestamp = time;
          if (msg[4] == 0x40)
            crude_buscan_data->data_t8.freno_servicio = 1;
          else
            crude_buscan_data->data_t8.freno_servicio = 0;
          freno_aux = msg[6] >> 4; //quedarse con la parte alta de D6
          //crude_buscan_data->data_t8.freno_servicio_intensidad = (double)((((msg[5]*16) + freno_aux)-552)/14.47);
          crude_buscan_data->data_t8.freno_servicio_intensidad = (double)((((msg[5]*16) + freno_aux)-552));
          break;
        case 1037: //ID Distancia
          crude_buscan_data->data_t9.timestamp = time;
          break;
        case 1042: //ID FRENO SERVICIO Y DE ESTACIONAMIENTO, PUERTAS ABIERTAS , TEMP MOTOR
          crude_buscan_data->data_t10.timestamp = time;
          //FRENO ESTACIONAMIENTO
          if ((msg[0] & 0x0F) == 0x00) //freno estacionamiento suelto
            crude_buscan_data->data_t10.freno_estacionamiento=0;
          else
            crude_buscan_data->data_t10.freno_estacionamiento=1;
          //PUERTAS ABIERTAS
          crude_buscan_data->data_t10.puertas_abiertas = msg[6] >> 3;
          //TEMPERATURA MOTOR
          crude_buscan_data->data_t10.temp_motor = (msg[5]-45);
          break;
        case 1074: //ID
          crude_buscan_data->data_t11.timestamp = time;
          break;
        case 1101: //ID VELOCIDAD ABS MEDIA delanteras y traseras por separado
          crude_buscan_data->data_t12.timestamp = time;
          crude_buscan_data->data_t12.vel_media_ruedas_delanteras = ((double)(msg[0]*256+msg[1])*0.01);
          crude_buscan_data->data_t12.vel_rueda_trasera_izq = ((double)(msg[2]*256+msg[3])*0.01);
          crude_buscan_data->data_t12.vel_rueda_trasera_dcha = ((double)(msg[4]*256+msg[5])*0.01);
          crude_buscan_data->data_t12.reservado = ((double)(msg[6]*256+msg[7])*0.01);
          break;
        case 1128: //ID
          crude_buscan_data->data_t13.timestamp = time;
          break;
        case 1160: //ID TEMPERATURA MOTOR
          crude_buscan_data->data_t14.timestamp = time;
          break;
        case 1161: //ID MARCHAS
          crude_buscan_data->data_t15.timestamp = time;
          //Modo cambio marchas
          if(msg[1]==12)
          {crude_buscan_data->data_t15.modo_cambio = invierno;}
          else if (msg[1]==4)
          {crude_buscan_data->data_t15.modo_cambio = sport;}
          else
          {crude_buscan_data->data_t15.modo_cambio = normal;}
          //Marchas

          switch (msg[0])
          {
            case 0xA0: crude_buscan_data->data_t15.marchas = P;	  break;
            case 0x91: crude_buscan_data->data_t15.marchas = R;	  break;
            case 0xA2: crude_buscan_data->data_t15.marchas = N;	  break;
            case 0x13: crude_buscan_data->data_t15.marchas = D1;  break;
            case 0x23: crude_buscan_data->data_t15.marchas = D2;	break;
            case 0x33: crude_buscan_data->data_t15.marchas = D3;	break;
            case 0x43: crude_buscan_data->data_t15.marchas = D4;	break;
            case 0x17: crude_buscan_data->data_t15.marchas = M1;	break;
            case 0x26: crude_buscan_data->data_t15.marchas = M2;	break;
            case 0x35: crude_buscan_data->data_t15.marchas = M3;	break;
            case 0x44: crude_buscan_data->data_t15.marchas = M4;	break;
            default:   crude_buscan_data->data_t15.marchas = N;	  break;
          }

          //printf("marcha=%x data_t15.marchas= %d\n", msg[0], crude_buscan_data->data_t15.marchas);
          break;
        case 1293: //ID DISTANCIAS en D1D2 y D2D3
          crude_buscan_data->data_t16.timestamp = time;
          break;
        case 1294: //ID
          crude_buscan_data->data_t17.timestamp = time;
          break;
        case 1362: //ID TIMER 1 seg.
          crude_buscan_data->data_t18.timestamp = time;
          break;
        case 1416: //ID
          crude_buscan_data->data_t19.timestamp = time;
          break;
        case 1426: //ID Limpiaparabrisas delantero.
          crude_buscan_data->data_t20.timestamp = time;
          //LIMPIAPARABRISAS
          switch (msg[2])
          {
            case 0x00: crude_buscan_data->data_t20.limpiaparabrisas = 0; break;
            case 0x10: crude_buscan_data->data_t20.limpiaparabrisas = 1; break;
            case 0x20: crude_buscan_data->data_t20.limpiaparabrisas = 2; break;
            case 0x40: crude_buscan_data->data_t20.limpiaparabrisas = 3; break;
            case 0x08: crude_buscan_data->data_t20.limpiaparabrisas = 8; break;
            default: 	break;
          }
          break;
        case 1544: //ID 3 TRAMAS ??
          crude_buscan_data->data_t21.timestamp = time;
          break;
        case 1554: //ID LUCES, INTERMITENTES, NIVEL COMBUSTIBLE
          crude_buscan_data->data_t22.timestamp = time;
          //LUCES
          if ((msg[1]&0x04) > 0) //largas activadas
          {
            if ((msg[1]&0x02) > 0) //largas + cortas + posicion activadas
              crude_buscan_data->data_t22.luces = luces_largas;
            else if ((msg[1]&0x01) > 0) //largas + posicion activadas
              crude_buscan_data->data_t22.luces = luces_rafaga_posicion;
            else //solo largas activadas = rafaga
              crude_buscan_data->data_t22.luces = luces_rafaga;
          }
          else if ((msg[1]&0x02) > 0) //cortas activadas
            crude_buscan_data->data_t22.luces = luces_cortas;
          else if ((msg[1]&0x01) > 0) //posicion activadas
            crude_buscan_data->data_t22.luces = luces_posicion;
          else
            crude_buscan_data->data_t22.luces = luces_off;

          if ((msg[1]&0x08) > 0) //antinieblas activadas
            crude_buscan_data->data_t22.luces_antiniebla = antiniebla_on;
          else
            crude_buscan_data->data_t22.luces_antiniebla = antiniebla_off;
          //INTERMITENTES
          intermitentes_aux = msg[1]&0xF0;
          switch (intermitentes_aux)
          {
            case 0x20:
              crude_buscan_data->data_t22.intermitente = intermitente_off;
              break;
            case 0xA0:
              crude_buscan_data->data_t22.intermitente = intermitente_izquierda;
              break;
            case 0x60:
              crude_buscan_data->data_t22.intermitente = intermitente_derecha;
              break;
            default:
              break;
          }
          //NIVEL COMBUSTIBLE
          crude_buscan_data->data_t22.nivel_combustible = (msg[3]*0.5);

          /*//CONSUMO MEDIO E INSTANTANEO
                  if (nivel_comb_inicial == 0.0)
                  {
                  //printf("nivel_comb_inicial ==%f",crude_buscan_data->data_t22.nivel_combustible );
                      nivel_comb_inicial = crude_buscan_data->data_t22.nivel_combustible;
                      nivel_comb_anterior = crude_buscan_data->data_t22.nivel_combustible;
                  }
                  if (nivel_comb_anterior != crude_buscan_data->data_t22.nivel_combustible);
                  {
                  crude_buscan_data->data_t22.consumo_medio = (nivel_comb_inicial - crude_buscan_data->data_t22.nivel_combustible)/(crude_buscan_data->distancia*100000);
                  crude_buscan_data->consumo_instantaneo = (nivel_comb_anterior - crude_buscan_data->nivel_combustible) /(crude_buscan_data->distancia - distancia_comb_anterior);
                  distancia_comb_anterior = crude_buscan_data->distancia; //Actualizar la distancia
                  }*/

          break;
        case 1928: //ID
          crude_buscan_data->data_t23.timestamp = time;
          break;
        case 1929: //ID
          crude_buscan_data->data_t24.timestamp = time;
          break;
        case 1933: //ID
          crude_buscan_data->data_t25.timestamp = time;
          break;
        case 1938: //ID
          crude_buscan_data->data_t26.timestamp = time;
          break;
        case 2018: //ID TEMPERATURA EXTERIOR
          crude_buscan_data->data_t27.timestamp = time;
          //TEMPERATURA EXTERIOR
          if (msg[2]==0x8D)
            crude_buscan_data->data_t27.temp_ext=(double)((msg[3]-80)*0.5);
          //else
          //	crude_buscan_data->data_t27.temp_ext=-100; //Para evitar valores extraños al principio y evitar valores incorrectos cuando no hay lectura
          break;
        case 2034: //ID TEMPERATURA EXTERIOR
          crude_buscan_data->data_t28.timestamp = time;
          break;
        default:
          break;
      } 
    }

    rclcpp::Subscription<buscan_msgs::msg::CanRaw>::SharedPtr subscription_;
    rclcpp::Publisher<buscan_msgs::msg::CanMsg>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanParser>());
  rclcpp::shutdown();
  return 0;
}
