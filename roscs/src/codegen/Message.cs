
using System;
using System.IO;
using System.Collections.Generic;
using System.Diagnostics;
namespace CSCodeGen
{


	public class Message
	{
		//static int idcounter = 1;
		public string BaseName {get; protected set;}
		public string NameSpace {get; protected set;}
		public string FullName {get; protected set;}
		public ulong Id {get; protected set;}
		
		protected List<MessageField> fields;
		protected List<Constant> constants;
		
		public static Dictionary<string,string> PointersUsed = new Dictionary<string, string>();
		
		public const int GEN_CPP = 1;
		public const int GEN_CS = 2;
		public static int Mode;
		
		public Message(){}
		public Message (string qualifiedName){
			int lastSlash = qualifiedName.LastIndexOf('/');
			if (lastSlash < 0) {
				//throw new Exception("unqualified Message Name: "+qualifiedName);
				Console.WriteLine("No Namespace given, assuming std_msgs");
				this.BaseName = qualifiedName;
				this.NameSpace = "std_msgs/";
				this.FullName = "std_msgs/"+qualifiedName; //use given name, since this is referred to
			} else {		
				this.BaseName = qualifiedName.Substring(lastSlash+1);
				this.NameSpace = qualifiedName.Substring(0,lastSlash+1);
				this.FullName = qualifiedName;
			}
			Console.WriteLine("Message: Name: {0} NameSpace: {1}",BaseName,NameSpace);
			
			//this.Id = idcounter++;
			List<string> content = GetMessageDescription(this.NameSpace+this.BaseName);
						
			this.fields = new List<MessageField>();
			this.constants = new List<Constant>();
			
			foreach(string s in content) {
				if (s.Contains("=")) {
					this.constants.Add(new Constant(s));
				}
				else {
					this.fields.Add(new MessageField(s));
				}
			}
			foreach(MessageField mf in this.fields) {
				if (this.BaseName.ToLower() == mf.Name.ToLower()) {				
					mf.Name = "the"+MessageField.FirstLetterToUpper(mf.Name);					
				}
			}
			this.Id = GetOwnId();
		}
		public List<string> RequiredMessages() {
			List<string> ret = new List<string>();
			foreach(MessageField mf in this.fields) {
				string name;
				if(mf.RequiresMessage(out name) && !ret.Contains(name)) {
					ret.Add(name);
				}
			}
			return ret;			
		}
		public void SetReferences(Dictionary<string,Message> msges) {
			foreach(MessageField mf in this.fields) {
				mf.SetReferences(msges);
			}
		}
		public string RosPackage {
			get {
				int idx = this.NameSpace.Substring(0,this.NameSpace.Length-1).LastIndexOf('/');
				if (idx < 0) {
					return this.NameSpace.Substring(0,this.NameSpace.Length-1);					
				} else {
					string ret = this.NameSpace.Substring(idx+1);
					return ret.Substring(0,ret.Length-1);
				}
			}
		}
		public string RosClassName {
			get {
				return this.FullName.Replace("/","::");				
			}
		}
		public string CsClassName {
			get {
				return this.FullName.Replace("/",".");				
			}
		}
		public string CsNameSpace {
			get {
				if (this.NameSpace=="") return "RosCS";
				else return "RosCS."+this.NameSpace.Replace("/",".").Substring(0,this.NameSpace.Length-1);				
			}
		}
		public string GetCSFields() {
			string ret ="public override ulong MessageType{ get { return "+this.Id+";}}\n";
			ret += "public const ulong TypeId = "+this.Id+";\n";
			if (this.constants.Count > 0) {
				ret += "\n#region Constants\n\n";			
				foreach(Constant cf in this.constants) {
					ret += cf.GetCSDeclaration();
				}
				ret += "\n#endregion\n\n";			

			}
			ret += "\n#region Properties\n\n";			
			foreach(MessageField mf in this.fields) {
				ret += mf.GetCSDeclaration();
			}
			ret += "\n#endregion\n\n";			
			return ret;
			
		}
		public string GetCSConstructorBody() {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				ret += mf.GetCSInitialisation();
			}
			return ret;
		}
		public string GetCSSizeBody() {
			int i=0;
			string bd="";
			foreach(MessageField mf in this.fields) {
				int tmp = mf.GetSize();
				if (tmp > 0) {
					i+= tmp;
				} else {
					bd += mf.GetCSSizeString()+"\n";
				}
			}
			string ret = "int ret = "+i+";\n";
			ret+= bd;			
			return ret+"return ret;\n";
		}
		public string GetCPPSizeBody() {
			int k=0;
			int i=0;
			string bd="";
			foreach(MessageField mf in this.fields) {
				int tmp = mf.GetSize();
				if (tmp > 0) {
					k+= tmp;
				} else {					
					
					bd += mf.GetCPPSizeString(i)+"\n";
					i++;
				}
			}
			string ret = "uint32_t ret = "+k+";\n";
			ret+= bd;			
			return ret+"return ret;\n";
		}
		public string Ros2Char(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				ret += mf.Ros2Char(indirection, ref ptrType,ref ptrName, depth);
			}
			return ret;
		}
		public string Char2Ros(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				ret += mf.Char2Ros(indirection, ref ptrType,ref ptrName, depth);
			}
			return ret;
		}
		public string FromByte(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				ret += mf.FromByte(indirection, ref ptrType,ref ptrName, depth);
			}
			return ret;
		}
		public string ToByte(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			foreach(MessageField mf in this.fields) {
				ret += mf.ToByte(indirection, ref ptrType,ref ptrName, depth);
			}
			return ret;
		}
		public string PtrLoopFix(ref string ptrType, ref string ptrName) {
			return this.fields[0].PtrLoopFix(ref ptrType,ref ptrName,false);			
		}
		public HashSet<string> BaseTypes() {
			HashSet<string> ret = new HashSet<string>();
			foreach(MessageField mf in this.fields) {
				List<string> pts = mf.BaseTypes();
				foreach(string s in pts) {
					if(!ret.Contains(s)) ret.Add(s);
				}
			}
			return ret;
		}
		public string PointerDeclaration(params string[] haveType) {
			HashSet<string> types = BaseTypes();
			string ret = "";
			foreach(string type in types) {
				bool have = false;
				foreach(string ht in haveType) {
					if (ht.Equals(type)) {
						have=true;
						break;
					}
				}
				if(!have) {
					if(Mode == GEN_CS) {
						ret += MessageField.baseTypeMapping[type].A+"* "+type+"p;\n";
					} else {
						ret += MessageField.baseTypeMapping[type].B+"* "+type+"p;\n";
					}
				}
			}
			return ret;
		}
		static List<string> GetMessageDescription(string msgName) {
			Process plist = new Process();			
			ProcessStartInfo psi = new ProcessStartInfo("rosmsg", "show "+msgName);
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
			List<string> ret = new List<string>();
			foreach(string s in strarr) {				
				if(s.StartsWith("ERROR")) {
					throw new Exception("Problem with message: "+msgName+"\t"+s);
				}
				if(s.StartsWith("Unknown msg type")) {
					throw new Exception(s);
				}
				if(!String.IsNullOrEmpty(s) && !s.StartsWith(" ") && !s.StartsWith("\t")) {
					//Console.WriteLine(msgName + ":" + s);		
					ret.Add(s);
				}				
			}
			return ret;
			
		}
		
		public ulong GetOwnId() {
			/*Process plist = new Process();
			ProcessStartInfo psi = new ProcessStartInfo("rosmsg", "md5 "+this.FullName);
			plist.EnableRaisingEvents = false;
			
			
			psi.UseShellExecute = false;
			psi.ErrorDialog = false;
			psi.RedirectStandardError = true;
			psi.RedirectStandardOutput = true;
			plist.StartInfo = psi;		
			
			plist.Start();
			
			string ret = plist.StandardOutput.ReadToEnd();
			plist.WaitForExit();
			plist.Close();
			int idx = ret.LastIndexOf(":");
			ret = ret.Substring(idx+1).Trim();
			string l = ret.Substring(0,16);
			string r = ret.Substring(16);
			ulong hash = ulong.Parse(l,System.Globalization.NumberStyles.AllowHexSpecifier)^ulong.Parse(r,System.Globalization.NumberStyles.AllowHexSpecifier);
			hash^=(ulong)this.FullName.GetHashCode();*/
			ulong hash = (ulong)this.FullName.GetHashCode();
			return hash;
		}
	}
}
