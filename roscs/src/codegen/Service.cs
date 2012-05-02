
using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
namespace CSCodeGen
{


	public class Service : Message
	{
	
		public Service(){}
		public Service (string qualifiedName, List<string> content){
			int lastSlash = qualifiedName.LastIndexOf('/');
			if (lastSlash < 0) {
				//throw new Exception("unqualified Message Name: "+qualifiedName);
				Console.WriteLine("No Namespace given, assuming std_msgs");
				base.BaseName = qualifiedName;
				base.NameSpace = "std_msgs/";
				base.FullName = "std_msgs/"+qualifiedName; //use given name, since this is referred to
			} else {		
				base.BaseName = qualifiedName.Substring(lastSlash+1);
				base.NameSpace = qualifiedName.Substring(0,lastSlash+1);
				base.FullName = qualifiedName;
			}
			Console.WriteLine("Message: Name: {0} NameSpace: {1}",BaseName,NameSpace);
			
			base.fields = new List<MessageField>();
			base.constants = new List<Constant>();
			
			foreach(string s in content) {
				if (s.Contains("=")) {
					base.constants.Add(new Constant(s));
				}
				else {
					base.fields.Add(new MessageField(s));
				}
			}
			foreach(MessageField mf in this.fields) {
				if (base.BaseName.ToLower() == mf.Name.ToLower()) {				
					mf.Name = "the"+MessageField.FirstLetterToUpper(mf.Name);					
				}
			}
			base.Id = GetOwnId();
		}
		
		public static List<List<string>> GetServiceDescription(string msgName) {
			Process plist = new Process();			
			ProcessStartInfo psi = new ProcessStartInfo("rossrv", "show "+msgName);
			plist.EnableRaisingEvents = false;
			
			psi.UseShellExecute = false;
			psi.ErrorDialog = false;
			psi.RedirectStandardError = true;
			psi.RedirectStandardOutput = true;
			plist.StartInfo = psi;		

			plist.Start();
			
			string msgDescr = plist.StandardOutput.ReadToEnd();
			string msgErr = plist.StandardError.ReadToEnd();
			
			plist.WaitForExit();
			//plist.Close();
			if (!String.IsNullOrEmpty(msgErr)) {
				throw new Exception("Problem while quering ros for msg "+msgName+" :\n"+msgErr);
			}
			string[] strarr = msgDescr.Split('\n');
			List<List<string>> ret = new List<List<string>>();
			List<string> request = new List<string>();
			List<string> response = new List<string>();
			bool requestFlag = true;
			foreach(string s in strarr) {
				if(s.StartsWith("ERROR")) {
					throw new Exception("Problem with message: "+msgName+"\t"+s);
				}
				if(s.StartsWith("Unknown msg type")) {
					throw new Exception(s);
				}
				if(!String.IsNullOrEmpty(s) && !s.StartsWith(" ") && !s.StartsWith("\t")) {
					
					if( s.Equals("---") ) {
						requestFlag = false;
						continue;
					}
					
					if( requestFlag ) {
						request.Add(s);
					} else {
						response.Add(s);
					}
				}				
			}
			ret.Add(request);
			ret.Add(response);
			return ret;
			
		}
		
		public string FromByteResponse(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				if( mf.OriginalRosName.Equals("response") ) {
					ret += mf.FromByte(indirection, ref ptrType,ref ptrName, depth);
					return ret;
				}
			}
			return "";
		}
		
		public string FromByteRequest(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				if( mf.OriginalRosName.Equals("request") ) {
					ret += mf.FromByte(indirection, ref ptrType,ref ptrName, depth);
					return ret;
				}
			}
			return "";
		}
		
		public string ToByteResponse(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				if( mf.OriginalRosName.Equals("response") ) {
					ret += mf.ToByte(indirection, ref ptrType,ref ptrName, depth);
				}
			}
			return ret;
		}
	}
}
