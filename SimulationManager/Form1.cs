using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using VISSIMLIB;

namespace CACCManGUI
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            List<int> MainVolInput = new List<int>();
            List<int> OnRampVolInput = new List<int>();
            List<int> OutGoingLinks = new List<int>();
            List<int> InComingLinks = new List<int>();
            List<int> MainVol = new List<int>();
            List<int> OnRampVol = new List<int>();
           

            StreamWriter sw = new StreamWriter(Directory.GetParent(textBox13.Text) + "\\CACCConf.dat");

            sw.WriteLine(Math.Round(Convert.ToDouble(textBox1.Text) / 3.0,1)); // ft/s -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox2.Text) / 3.0, 1)); // ft/s -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox3.Text) / 3.0, 1)); // ft/s -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox4.Text) / 3.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox5.Text) * 1.604/3.6, 1)); // mi/h -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox6.Text) * 1.604/3.6, 1)); // mi/h -> m/s
            sw.WriteLine(textBox7.Text);
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox8.Text) / 3.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox9.Text) / 3.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox10.Text) / 1.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox11.Text) / 1.0, 1));
            sw.WriteLine(textBox47.Text); // Staging Link


            sw.Close();

            string fn = textBox12.Text;
            string fnini = textBox13.Text;
            int sp = int.Parse(textBox14.Text);
            int NumRep = int.Parse(textBox15.Text);
            int rseed = int.Parse(textBox16.Text);
            string maininput = textBox18.Text;
            string onrampinput = textBox19.Text;
            string outgoinglinks = textBox20.Text;
            string incominglinks = textBox21.Text;
            string Mainvol = textBox42.Text;
            string OnRVol = textBox43.Text;
            string MPCacc = textBox44.Text;
            string MPAcc = textBox45.Text;
            string MPHov = textBox46.Text;

            MainVolInput = GetList(MainVolInput, maininput);
            OnRampVolInput = GetList(OnRampVolInput, onrampinput);
            OutGoingLinks = GetList(OutGoingLinks, outgoinglinks);
            InComingLinks = GetList(InComingLinks, incominglinks);
            MainVol = GetList(MainVol, Mainvol);
            OnRampVol = GetList(OnRampVol, OnRVol);

            for (int i = 1; i <= NumRep; i++)
            {
                RunCACCSimSingle(fn, fnini, sp, 10, rseed, MainVolInput, OnRampVolInput, OutGoingLinks, InComingLinks, MainVol, OnRampVol, MPCacc, MPAcc, MPHov);
                rseed++;
            }
        }


        private void label24_Click()
        {
        
        }

        private void textBox12_TextChanged()
        {
        
        }

        private void RunCACCSimSingle(string fn, string fnini, int SimTime, int simresol, int rseed, List<int> MainVolInput, List<int> OnRampVolInput, List<int> OutGoingLinks, List<int> InComingLinks, List<int> MainVol, List<int> OnRampVol, string MPCacc, string MPAcc, string MPHov)
        {
            int outci = 0; //initialized the counter for traffic on on-ramp 
            int inci = 0; //initialized the time coutner for traffic on off-ramp
            int ininterval = 2 * simresol; // In the on-ramp (shorter length) collect data at twice of the simulation interval
            int outinterval = 4 * simresol; // In the off-ramp (longer length) collect data at four times of the simulation interval
            int vid = 0;
            int type = 0;
            object VTYPE = null;
            object VIDs = null;

            //invoking the initiallizatio of the Vssim network based on the pararmeter defined above
            VissimTools.InitVissim(fn, fnini, SimTime, simresol, rseed);

            //seting the vehicle input volumes
            UpdateVolume(MainVolInput,MainVol); // Update Mainline Volume 
            UpdateVolume(OnRampVolInput, OnRampVol); // Update On-Ramp Volume

            //Setting the relative flow rate of CACC in the main input point and 1st on-ramp
            //VissimTools.SetRelFlow(2, "Relflow(100,1)", 1.0-Convert.ToDouble(MPCacc)); //vehicle compisiton row 2 (input #2), subatribute (100 vehicle type)
            //VissimTools.SetRelFlow(2, "Relflow(102,1)", Convert.ToDouble(MPCacc)); //vehicle compisiton row 2 (input #2), subatribute (102 vehicle type)
            //VissimTools.SetRelFlow(4, "Relflow(100,1)", 1.0-Convert.ToDouble(MPCacc)); //vehicle compisiton row 4 (input #4), subatribute (100 type and 101 type)
            //VissimTools.SetRelFlow(4, "Relflow(101,1)", Convert.ToDouble(MPCacc)); //vehicle compisiton row 4 (input #4), subatribute (101 vehicle type)

            // TO DO : We need to add ACC mode Market Penetration Rates; Traffic Composition ID must be obatined through the GUI.

            for (int i = 1; i <= simresol * SimTime; i++)
            {
                VissimTools.vissim.Simulation.RunSingleStep();

                if (i == 845)
                    Console.Write("!");

                inci++;
                outci++;

                if (inci == ininterval)
                {
                    foreach (int val in InComingLinks)
                    {

                        VIDs = VissimTools.GetLinkVehiclesbyNumber(val);
                        VTYPE = VissimTools.GetLinkVehiclesByType(val);

                        for (int k = 0; k < ((object[,])(VIDs)).Length / 2; k++)
                        {
                            type = Convert.ToInt32(((object[,])(VTYPE))[k, 1]);

                            if (type == 102)
                            {
                                vid = Convert.ToInt32(((object[,])(VIDs))[k, 1]);

                                if (VissimTools.Get_Lane(vid) > 1)
                                {
                                    VissimTools.Set_VehType(vid, 101);
                                }
                            }
                        }

                        inci = 0;
                    }
                }

                if (outci == outinterval)
                {
                    foreach (int val in OutGoingLinks)
                    {
                        VIDs = VissimTools.GetLinkVehiclesbyNumber(val);
                        VTYPE = VissimTools.GetLinkVehiclesByType(val);

                        for (int k = 0; k < ((object[,])(VIDs)).Length / 2; k++)
                        {
                            type = Convert.ToInt32(((object[,])(VTYPE))[k, 1]);

                            if (type == 101)
                            {
                                vid = Convert.ToInt32(((object[,])(VIDs))[k, 1]);

                                if (VissimTools.GetRoute(vid) == 2)
                                {
                                    VissimTools.Set_VehType(vid, 100);
                                }
                            }
                        }

                        outci = 0;
                    }


                }

            }
        }

        private void UpdateVolume(List<int> VolInput, List<int> Vol)
        {
            int p = 0;

            foreach (int val in VolInput)
            {
                VissimTools.SetVol(val, "Volume(1)", Vol[p]); //vehicle input point [VolInput], volume of 1st interval 
                //VissimTools.SetVol(val, "Volume(2)", Vol); //vehicle input point [VolInput], volume of 2nd interval 
                p++;
            }
        }

        private static List<int> GetList(List<int> List_, string str)
        {
            string[] tmp = null;

            tmp = str.Split(",".ToCharArray());

            for (int i = 0; i < tmp.Length; i++)
            {
                if (tmp[i].Length>0)
                    List_.Add(int.Parse(tmp[i]));
            }
            return List_;
        }


        private void button2_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }

        private void button3_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
            string str1 = openFileDialog1.FileName;
            textBox12.Text = str1;
        }

        private void button4_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
            string str1 = openFileDialog1.FileName;
            textBox13.Text = str1;
        }

        private void Form1_Load(object sender, EventArgs e)
        {

        }

        private void button6_Click(object sender, EventArgs e)
        {
            StreamWriter sw = new StreamWriter(Directory.GetParent(textBox13.Text) + "\\CACCConf.dat");

            sw.WriteLine(Math.Round(Convert.ToDouble(textBox1.Text) / 3.0, 1)); // ft/s -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox2.Text) / 3.0, 1)); // ft/s -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox3.Text) / 3.0, 1)); // ft/s -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox4.Text) / 3.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox5.Text) * 1.604 / 3.6, 1)); // mi/h -> m/s
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox6.Text) * 1.604 / 3.6, 1)); // mi/h -> m/s
            sw.WriteLine(textBox7.Text);
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox8.Text) / 3.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox9.Text) / 3.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox10.Text) / 1.0, 1));
            sw.WriteLine(Math.Round(Convert.ToDouble(textBox11.Text) / 1.0, 1));


            sw.Close();

            string fn = textBox12.Text;
            string fnini = textBox13.Text;
            int sp = int.Parse(textBox14.Text);
            int res = int.Parse(textBox15.Text);
            int rseed = int.Parse(textBox16.Text);
            //RunCACCSim(fn, fnini, sp, res, rseed);
        }

        private void tabPage1_Click(object sender, EventArgs e)
        {

        }

        private void button9_Click(object sender, EventArgs e)
        {
            Application.Exit();
        }
    }


    class VissimTools
    {
        static public Vissim vissim = new Vissim();

        static public void InitVissim(string fn, string fn_ini, int sp, int res, int rseed)
        {
            vissim.LoadNet(fn, false);
            vissim.LoadLayout(fn_ini);
            vissim.Simulation.set_AttValue("SimPeriod", sp);
            vissim.Simulation.set_AttValue("SimRes", res);
            vissim.Simulation.set_AttValue("RandSeed", rseed);

        }

        static public void EndVissim()
        {
            vissim.Exit();
        }


        //static public void Set_InputVol(int id, int vol) { vissim.Net.VehicleInputs.get_ItemByKey(id).set_AttValue("Volume", vol); } // not ok
        public static object GetLinkVehiclesbyNumber(int lkn) { return vissim.Net.Links.get_ItemByKey(lkn).Vehs.get_GetMultiAttValues("No"); } // OK
        public static object GetAllVehiclesByType() { return vissim.Net.Vehicles.get_GetMultiAttValues("VehType"); } // OK
        public static object GetLinkVehiclesByType(int lkn) { return vissim.Net.Links.get_ItemByKey(lkn).Vehs.get_GetMultiAttValues("VehType"); } // OK
        public static object GetAllVehicles() { return VissimTools.vissim.Net.Vehicles.get_GetMultiAttValues("No"); } //OK
        static public int Get_VehType(int vid) { return (int)vissim.Net.Vehicles.get_ItemByKey(vid).get_AttValue("VehType"); } //OK
        static public int Get_Lane(int vid)
        {
            string tmp = null;
            int lan_ = 0;
            tmp = vissim.Net.Vehicles.get_ItemByKey(vid).get_AttValue("Lane");
            lan_ = int.Parse(tmp.Split("-".ToCharArray())[1]);
            return lan_;
        } //OK
        static public void Set_VehType(int vid, int type) { vissim.Net.Vehicles.get_ItemByKey(vid).set_AttValue("VehType", type); }
        static public int GetRoute(int vid) { return (int)vissim.Net.Vehicles.get_ItemByKey(vid).get_AttValue("RouteNo"); }
        static public void SetVol(int id, string volbin, int vol) { vissim.Net.VehicleInputs.get_ItemByKey(id).set_AttValue(volbin, vol); }
        static public void SetRelFlow(int id, string attr, double fr) { vissim.Net.VehicleCompositions.get_ItemByKey(id).set_AttValue(attr, fr); }

    }
}
