using System;
using System.IO;
//using C5;
//using ros;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text.RegularExpressions;
namespace CSCodeGen
{



	public class DepQuery
	{
		
		
		static void Main(string[] args)
		{

			
			
			if( args.Length < 1 || args.Length > 2 )
			{
				Console.WriteLine("Usage QueryDependencies.exe [options] <packageName>");
				Console.WriteLine("-m for message (standard)");
				Console.WriteLine("-s for service (standard)");
				return;
			}
			
			string packageName = "";
			if( args.Length == 1 )
				packageName = args[0];
			
			bool message = true;
			if( args.Length == 2 )
			{
				string option = args[0];
				packageName = args[1];
				
				if( option.Equals("-s") )
					message = false;
			}
			
            Process myProcess = new Process();
            
            ProcessStartInfo myProcessStartInfo = new ProcessStartInfo("rospack");
            myProcessStartInfo.UseShellExecute = false;
            myProcessStartInfo.RedirectStandardOutput = true;
            myProcess.StartInfo = myProcessStartInfo;
			
			if( message )
				myProcess.StartInfo.Arguments = "depends " + packageName;
			else
				myProcess.StartInfo.Arguments = "depends1 " + packageName;
            myProcess.Start();
            StreamReader myStreamReader = myProcess.StandardOutput;
            // Read the standard output of the spawned process.
            string deps = myStreamReader.ReadToEnd();
            myProcess.WaitForExit();
            myProcess.Close();
			
			List<string> packages = new List<string>();
			packages.Add(packageName);
			
			string[] strarr = deps.Split('\n');
			foreach(string s in strarr) {
				string st = s.Trim();
				if (st!="") packages.Add(st);
			}
			/*if( !packages.Contains("std_msgs") )
			{
				packages.Add("std_msgs");
			}*/
			List<string> msgFiles = new List<string>();
			bool first = true;
			foreach(string p in packages) {
				string[] files;
				if( message )
					files = MessagesOfPackage(p);
				else
					files = ServicesOfPackage(p);
				foreach(string f in files) {
					if (first) {
						first=false;
					} else {
						Console.Write(";");
					}
					Console.Write(f);
				}							
			}
			
			
		}
		
		
		public static string[] MessagesOfPackage(string package) {
			Process plist = new Process();
			ProcessStartInfo psi = new ProcessStartInfo("rospack", "find "+package);
			plist.EnableRaisingEvents = false;
			
			
			psi.UseShellExecute = false;
			psi.ErrorDialog = false;
			psi.RedirectStandardError = true;
			psi.RedirectStandardOutput = true;
			plist.StartInfo = psi;		
			
			plist.Start();
			
			string path = plist.StandardOutput.ReadLine();			
			plist.WaitForExit();
			plist.Close();
			
			path = path.Trim();
			path +="/msg/";
			if (Directory.Exists(path)) {
				string[] files = Directory.GetFiles(path,"*.msg");
				return files;
			} else  return new string[0];
		}
		
		public static string[] ServicesOfPackage(string package) {
			Process plist = new Process();
			ProcessStartInfo psi = new ProcessStartInfo("rospack", "find "+package);
			plist.EnableRaisingEvents = false;
			
			
			psi.UseShellExecute = false;
			psi.ErrorDialog = false;
			psi.RedirectStandardError = true;
			psi.RedirectStandardOutput = true;
			plist.StartInfo = psi;		
			
			plist.Start();
			
			string path = plist.StandardOutput.ReadLine();			
			plist.WaitForExit();
			plist.Close();
			
			path = path.Trim();
			path +="/srv/";
			if (Directory.Exists(path)) {
				string[] files = Directory.GetFiles(path,"*.srv");
				return files;
			} else  return new string[0];
		}
		
	}
}
