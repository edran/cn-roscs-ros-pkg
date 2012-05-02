
using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace CSCodeGen
{


	public class ServiceField
	{
		string rosType;
		string atomicRosType;
		string name;
		
		public bool IsArrayType {get; private set;}
		public bool IsFixedSizeArray {get; private set;}
		public int ArraySize {get; private set;}
		public bool IsBaseType {get; private set;}
		public Service ServiceRef { get; private set;}
		public string Name {get { return this.name; } set { this.name = value; }}
		public string OriginalRosName{get; set;}
		
		public static Dictionary<string,TypeEntry> baseTypeMapping;
		
		public ServiceField (string def){

			if (baseTypeMapping == null) InitBaseTypeMapping();
			
			string[] strarr = def.Split(' ','\t');
			
			if (strarr.Length!=2) {
				throw new Exception("Unexpected message field format: "+def);
			}
			this.rosType = strarr[0].Trim();
			
			this.name = strarr[1].Trim();
			
			//HACK for HEADER:
			if (this.rosType == "Header") {
				this.rosType = "std_msgs/Header";
			}
			
			this.IsArrayType = this.rosType.Contains("[");
			if(this.IsArrayType) {
				this.atomicRosType = this.rosType.Substring(0,this.rosType.IndexOf('['));
				Regex fixedSizeRe = new Regex(@"\[([0-9]+)\]");
				if (fixedSizeRe.IsMatch(this.rosType)) {
					this.IsFixedSizeArray = true;
					this.ArraySize = Int32.Parse(fixedSizeRe.Match(this.rosType).Groups[1].Value);					
				} else {
					this.IsFixedSizeArray = false;
				}
				
				
			} else {
				this.atomicRosType = this.rosType;
			}
			
			this.IsBaseType = baseTypeMapping.ContainsKey(this.atomicRosType);
			this.ServiceRef = null;
			this.OriginalRosName = this.name;
			Console.WriteLine("Service Field: {0} - {1}",this.rosType,this.name);
		}
		
		public bool RequiresService(out string msg) {
			msg = null;
			if (this.IsBaseType) return false;
			msg = this.atomicRosType;
			return true;
		}
		
		public string GetCSDeclaration() {
			string type = GetCSType();
			//string field = "protected "+type + " "+FirstLetterToLower(this.name)+";\n";
			string prop ="\t\tpublic "+type+" "+FirstLetterToUpper(this.name) + " { get; set;}\n";
			
			return prop;
		}
		public string GetCSSizeString() {
			if (this.IsFixedSizeArray) {
				if (this.IsBaseType && this.atomicRosType != "string") {
					return "ret+= "+(this.ArraySize*baseTypeMapping[this.atomicRosType].Size)+";\n";
				}
				if (this.atomicRosType == "string") {
					string r = "foreach("+this.GetCSBaseType()+" e in this."+FirstLetterToUpper(this.name)+") {\n";
					r += "ret+= e.Length+4; //int length per string\n";
					return r+"}\n";
				}
				string s = "foreach("+this.GetCSBaseType()+" e in this."+FirstLetterToUpper(this.name)+") {\n";
				s += "ret+= e.GetSize();\n";
				return s+"}\n;";
			}
			if (this.IsArrayType) {
				if (this.IsBaseType && this.atomicRosType != "string") {
					string s = "ret+= 4 + "+FirstLetterToUpper(this.name)+".Count*"+baseTypeMapping[this.atomicRosType].Size+";\n";
					return s;
				}
				if (this.atomicRosType == "string") {
					string s = "foreach("+this.GetCSBaseType()+" e in this."+FirstLetterToUpper(this.name)+") {\n";
					s += "ret+= e.Length+4; //int length per string\n";
					return s+"}\n ret += 4; //int number of strings\n";
				}
			
				string r = "foreach("+this.GetCSBaseType()+" e in this."+FirstLetterToUpper(this.name)+") {\n";
				r += "ret+= e.GetSize();\n";
				return r+"}\n ret+= 4; //int number of "+this.GetCSBaseType()+"\n";
			
			}
			if (this.atomicRosType == "string") return "ret += "+FirstLetterToUpper(this.name)+".Length+4;";
			if (!this.IsBaseType) {
				return "ret += "+FirstLetterToUpper(this.name)+".GetSize();";
			}
			throw new Exception("Unexpected Type in GetCSSizeString");
		}
		public string GetCPPSizeString(int c) {
			if (this.IsFixedSizeArray) {
				if (this.IsBaseType && this.atomicRosType != "string") {
					return "ret+= "+(this.ArraySize*baseTypeMapping[this.atomicRosType].Size)+";\n";					
				}
				if (this.atomicRosType == "string") {
					string r = "for(size_t i=0; i < "+this.ArraySize+"; i++) {\n";
					r += "ret+= m."+this.OriginalRosName+"[i].size()+4; //int length per string\n";
					return r+"}\n";
				}
			
				string s = "for(size_t i=0; i < "+this.ArraySize+"; i++) {\n";
				s += "ret+= getSizeOf(m."+this.OriginalRosName+"[i]);\n";
				return s+"}\n";			
			}

			if (this.IsArrayType) {
				if (this.IsBaseType && this.atomicRosType != "string") {
					string s = "ret+= 4 + m."+this.OriginalRosName+".size()*"+baseTypeMapping[this.atomicRosType].Size+";";
					return s;
				}
				if (this.atomicRosType == "string") {
					string s = "for(size_t i=0; i < m."+this.OriginalRosName+".size(); i++) {\n";
					s += "ret+= m."+this.OriginalRosName+"[i].size()+4; //int length per string\n";
					return s+"}\n ret += 4; //int number of strings\n";
				}
			
				string r = "for(size_t i=0; i < m."+this.OriginalRosName+".size(); i++) {\n";
				r += "ret+= getSizeOf(m."+this.OriginalRosName+"[i]);\n";
				return r+"}\n ret+= 4; //int number of "+this.GetCSBaseType()+"\n";			
			}
			if (this.atomicRosType == "string") return "ret += m."+this.OriginalRosName+".size()+4;";
			if (!this.IsBaseType) {
				//string ret = this.ServiceRef.RosClassName+"::ConstPtr p"+c+" = "+this.ServiceRef.RosClassName+"::ConstPtr(&(m->"+this.name+"));\n";
				//return ret+"ret+= getSizeOf(p"+c+");\n";				
				return "ret+= getSizeOf(m."+this.OriginalRosName+");\n";				
			}
			throw new Exception("Unexpected Type in GetCSSizeString");
		}
		public int GetSize() {
			if(this.IsBaseType && !this.IsArrayType) return baseTypeMapping[this.atomicRosType].Size;
			else return -1;
		}
		public string Ros2Char(string indirection,ref string ptrType,ref string ptrName,int depth) {
			string ret = "";
			if (this.IsBaseType && !this.IsArrayType) {
				if (this.atomicRosType == "string") {
					//Length of string: int32_t
					ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".size();\n";
					ret +=ptrName+"++;\n";
					ret += SetPointerTo("char",ref ptrType, ref ptrName);
					ret += indirection+this.OriginalRosName+".copy("+ptrName+","+indirection+this.OriginalRosName+".size());\n";
					ret += ptrName+"+="+indirection+this.OriginalRosName+".size();\n";
				} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
					ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
					ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".sec;\n";
					ret +=ptrName+"++;\n";
					ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".nsec;\n";
					ret +=ptrName+"++;\n";
				}
				else {
					//simple case
					ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
					ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+";\n";
					ret +=ptrName+"++;\n";
				}
				return ret;
			}
			if (!this.IsArrayType) {
				return this.ServiceRef.Ros2Char(indirection+this.OriginalRosName+".",ref ptrType,ref ptrName,depth+1);
			}
			if (this.IsFixedSizeArray) {
				for(int i=0; i<this.ArraySize; i++) {
					if (this.IsBaseType) {
						if(this.atomicRosType == "string") {
							ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
							ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+"["+i+"].size();\n";
							ret += ptrName+"++;\n";
							ret += SetPointerTo("char",ref ptrType, ref ptrName);
							ret += indirection+this.OriginalRosName+"["+i+"].copy("+ptrName+","+indirection+this.OriginalRosName+"["+i+"].size());\n";					
							ret += ptrName +"+="+indirection+this.OriginalRosName+"["+i+"].size();\n";
							ret += SetPointerTo("int32",ref ptrType, ref ptrName);
						} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
							ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
							ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".sec;\n";
							ret +=ptrName+"++;\n";
							ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".nsec;\n";
							ret +=ptrName+"++;\n";
						}
						else {		 	
							ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
							ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+"["+i+"];\n";
							ret += ptrName+"++;\n";	
						}
					}
					else {
						ret += this.ServiceRef.Ros2Char(indirection+this.OriginalRosName+"["+i+"].",ref ptrType,ref ptrName,depth+1);
					}
				}
				return ret;
			}
			//Array Length:
			ret += SetPointerTo("int32",ref ptrType, ref ptrName);
			ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".size();\n";
			ret += ptrName+"++;\n";
			//loop:
			/*if (this.IsBaseType && this.atomicRosType != "string") {
				ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
			} else if (this.atomicRosType == "string") {
				ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
			}*/
			ret += PtrLoopFix(ref ptrType,ref ptrName,true);
			//Dictionary<string,string> usedPointers = new Dictionary<string, string>(Service.PointersUsed);
			ret += "for(size_t i"+depth+"=0; i"+depth+" < "+indirection+this.OriginalRosName+".size(); i"+depth+"++) {\n";
				if (this.IsBaseType) {
					if(this.atomicRosType == "string") {
						//ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
						ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+"[i"+depth+"].size();\n";
						ret += ptrName+"++;\n";
						ret += SetPointerTo("char",ref ptrType, ref ptrName);
						ret += indirection+this.OriginalRosName+"[i"+depth+"].copy("+ptrName+","+indirection+this.OriginalRosName+"[i"+depth+"].size());\n";
						ret += ptrName +"+="+indirection+this.OriginalRosName+"[i"+depth+"].size();\n";
						ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
						//ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
						ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".sec;\n";
						ret +=ptrName+"++;\n";
						ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+".nsec;\n";
						ret +=ptrName+"++;\n";
					}
					else {			
						//ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
						ret += "*"+ptrName+" = "+indirection+this.OriginalRosName+"[i"+depth+"];\n";
						ret += ptrName+"++;\n";	
					}
				}
				
				else {
					if( this.ServiceRef != null )
					{
						ret += this.ServiceRef.Ros2Char(indirection+this.OriginalRosName+"[i"+depth+"].",ref ptrType,ref ptrName,depth+1);
						ret += this.ServiceRef.PtrLoopFix(ref ptrType,ref ptrName);				
					}
				}
			ret +="}\n";			
			//Service.PointersUsed = usedPointers;
			return ret;
		}
		public string Char2Ros(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			if (this.IsBaseType && !this.IsArrayType) {
				if (this.atomicRosType == "string") {
					//Length of string: int32_t
					ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					ret += "int len_"+this.OriginalRosName+depth+" = (*"+ptrName+");\n";					
					ret += ptrName+"++;\n";
					ret += SetPointerTo("char",ref ptrType, ref ptrName);
					ret += indirection+this.OriginalRosName+ " = string("+ptrName+",len_"+this.OriginalRosName+depth+");\n";
					ret += ptrName+"+= len_"+this.name+depth+";\n";

				} else if (this.atomicRosType == "time") {
					ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
					ret += indirection+this.OriginalRosName+ " = ros::Time(*"+ptrName+",*("+ptrName+"+1));\n";					
					ret +=ptrName+"+=2;\n";
				} else if (this.atomicRosType == "duration") {
					ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
					ret += indirection+this.OriginalRosName+ " = ros::Duration(*"+ptrName+",*("+ptrName+"+1));\n";					
					ret +=ptrName+"+=2;\n";
				}
				else {
					//simple case
					ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
					ret += indirection+this.OriginalRosName+ " = (*"+ptrName+");\n";				
					ret +=ptrName+"++;\n";
				}
				return ret;
			}
			if (!this.IsArrayType) {
				//ret += indirection+FirstLetterToUpper(this.name)+" = new "+this.ServiceRef.CsClassName+"();\n";
				ret += this.ServiceRef.Char2Ros(indirection+this.OriginalRosName+".",ref ptrType,ref ptrName,depth+1);
				return ret;
			}
			if (this.IsFixedSizeArray) {
				if (this.atomicRosType == "string") {
					ret += "int len2_"+this.OriginalRosName+depth+";\n";
				}
				for(int i=0; i<this.ArraySize; i++) {
					if (this.IsBaseType) {
						if(this.atomicRosType == "string") {
							ret += SetPointerTo("int32",ref ptrType, ref ptrName);
							ret += "len2_"+this.OriginalRosName+depth+" = (*"+ptrName+");\n";					
							ret += ptrName+"++;\n";
							ret += SetPointerTo("char",ref ptrType, ref ptrName);
							ret += indirection+this.OriginalRosName+ "["+i+"]= string("+ptrName+",len2_"+this.OriginalRosName+depth+");\n";						
							ret += ptrName+" += len2_"+this.OriginalRosName+depth+";\n";							
						} else if (this.atomicRosType == "time") {
							ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
							ret += indirection+this.OriginalRosName+ "["+i+"] = ros::Time(*"+ptrName+",*("+ptrName+"+1));\n";					
							ret +=ptrName+"+=2;\n";
						} else if (this.atomicRosType == "duraton") {
							ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
							ret += indirection+this.OriginalRosName+ "["+i+"] = ros::Duration(*"+ptrName+",*("+ptrName+"+1));\n";					
							ret +=ptrName+"+=2;\n";
						}
						else {
							ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
							ret += indirection+this.OriginalRosName+"["+i+"] = (*"+ptrName+");\n";
							ret += ptrName+"++;\n";	
						}
					}
					else {
						ret += this.ServiceRef.Char2Ros(indirection+this.OriginalRosName+"["+i+"].",ref ptrType,ref ptrName,depth+1);
					}
				}
				return ret;
			}
			//Array Length:
			ret += SetPointerTo("int32",ref ptrType, ref ptrName);
			ret += "int len_"+this.OriginalRosName+depth+" = (*"+ptrName+");\n";

			ret += ptrName+"++;\n";
			if (this.atomicRosType == "string") {
				ret += "int len2_"+this.OriginalRosName+depth+";\n";
			}
			//loop:
			ret += PtrLoopFix(ref ptrType,ref ptrName,true);
			//Dictionary<string,string> usedPointers = new Dictionary<string, string>(Service.PointersUsed);
			ret += "for(int i"+depth+"=0; i"+depth+" < len_"+this.OriginalRosName+depth+"; i"+depth+"++) {\n";
				if (this.IsBaseType) {
					if(this.atomicRosType == "string") {					
						ret += "len2_"+this.OriginalRosName+depth+" = (*"+ptrName+");\n";					
						ret += ptrName+"++;\n";
						ret += SetPointerTo("char",ref ptrType, ref ptrName);
						ret += indirection+this.OriginalRosName+ ".push_back(string("+ptrName+",len2_"+this.OriginalRosName+depth+"));\n";						
						ret += ptrName+" += len2_"+this.OriginalRosName+depth+";\n";
						ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					} else if (this.atomicRosType == "time") {
						ret += indirection+this.OriginalRosName+ ".push_back(ros::Time(*"+ptrName+",*("+ptrName+"+1)));\n";
						ret +=ptrName+"+=2;\n";
					} else if (this.atomicRosType == "duraton") {
						ret += indirection+this.OriginalRosName+ ".push_back(ros::Duration(*"+ptrName+",*("+ptrName+"+1)));\n";					
						ret +=ptrName+"+=2;\n";
					}
					else {
						ret += indirection+this.OriginalRosName+".push_back(*"+ptrName+");\n";
						ret += ptrName+"++;\n";	
					}
				}
				
				else {					
					ret += indirection+this.OriginalRosName+".push_back("+this.ServiceRef.RosClassName+"());\n";
					ret += this.ServiceRef.Char2Ros(indirection+this.OriginalRosName+"[i"+depth+"].",ref ptrType,ref ptrName,depth+1);
					ret += this.ServiceRef.PtrLoopFix(ref ptrType,ref ptrName);				
				}
			ret +="}\n";			
			
			//Service.PointersUsed = usedPointers;
			
			return ret;
		}
		public string FromByte(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";
			if (this.IsBaseType && !this.IsArrayType) {
				if (this.atomicRosType == "string") {
					//Length of string: int32_t
					ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					ret += "int len_"+this.name+depth+" = (*"+ptrName+");\n";
					//ret += "uint l"+depth+"= (*"+ptrName+");\n"; 
					ret += ptrName+"++;\n";
					ret += SetPointerTo("char",ref ptrType, ref ptrName);
					ret += indirection+FirstLetterToUpper(this.name)+ " = new System.String("+ptrName+",0,len_"+this.name+depth+");\n";
					ret += ptrName+"+= len_"+this.name+depth+";\n";

				} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
					ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
					ret += indirection+FirstLetterToUpper(this.name)+" = (*"+ptrName+")*1000000000ul;\n";
					ret +=ptrName+"++;\n";
					ret += indirection+FirstLetterToUpper(this.name)+" += (*"+ptrName+");\n";					
					ret +=ptrName+"++;\n";
				}
				else {
					//simple case
					ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
					ret += indirection+FirstLetterToUpper(this.name)+ " = (*"+ptrName+");\n";				
					ret +=ptrName+"++;\n";
				}
				return ret;
			}
			if (!this.IsArrayType) {
				ret += indirection+FirstLetterToUpper(this.name)+" = new "+this.ServiceRef.CsClassName+"(false);\n";
				ret += this.ServiceRef.FromByte(indirection+FirstLetterToUpper(this.name)+".",ref ptrType,ref ptrName,depth+1);
				return ret;
			}
			if (this.IsFixedSizeArray) {
				if (this.atomicRosType == "string") {
				ret += "int len2_"+this.name+depth+";\n";
				}
				if (this.IsBaseType) {
					ret += indirection+FirstLetterToUpper(this.name) +" = new "+baseTypeMapping[this.atomicRosType].A+"["+this.ArraySize+"];\n";
				} else {
					ret += indirection+FirstLetterToUpper(this.name) +" = new "+this.ServiceRef.CsClassName+"["+this.ArraySize+"];\n";
				}
				for(int i=0; i<this.ArraySize; i++) {
					if (this.IsBaseType) {
						if(this.atomicRosType == "string") {
							ret += SetPointerTo("int32",ref ptrType, ref ptrName);
							ret += "len2_"+this.name+depth+" = (*"+ptrName+");\n";					
							ret += ptrName+"++;\n";
							ret += SetPointerTo("char",ref ptrType, ref ptrName);
							ret += indirection+FirstLetterToUpper(this.name)+ "["+i+"]=new System.String("+ptrName+",0,len2_"+this.name+depth+"));\n";	
							ret += ptrName+" += len2_"+this.name+depth+";\n";							
						} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
							ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
							ret += indirection+FirstLetterToUpper(this.name)+"["+i+"] = (*"+ptrName+")*1e9ul;\n";
							ret +=ptrName+"++;\n";
							ret += indirection+FirstLetterToUpper(this.name)+"["+i+"] += (*"+ptrName+");\n";					
							ret +=ptrName+"++;\n";
						}
						else {
							ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
							ret += indirection+FirstLetterToUpper(this.name)+"["+i+"] =(*"+ptrName+");\n";
							ret += ptrName+"++;\n";	
						}
					}
					else {
						ret += indirection+FirstLetterToUpper(this.name)+"["+i+"] = new "+this.ServiceRef.CsClassName+"(false);\n";
						ret += this.ServiceRef.FromByte(indirection+FirstLetterToUpper(this.name)+"["+i+"].",ref ptrType,ref ptrName,depth+1);												
					}
				}
				return ret;
			}
			//Array Length:
			ret += SetPointerTo("int32",ref ptrType, ref ptrName);
			ret += "int len_"+this.name+depth+" = (*"+ptrName+");\n";
			if (this.IsBaseType) {
				ret += indirection+FirstLetterToUpper(this.name) +" = new List<"+baseTypeMapping[this.atomicRosType].A+">();\n";				
			} else {
				ret += indirection+FirstLetterToUpper(this.name) +" = new List<"+this.ServiceRef.CsClassName+">();\n";				
			}
			ret += ptrName+"++;\n";
			if (this.atomicRosType == "string") {
				ret += "int len2_"+this.name+depth+";\n";
			}
			//loop:
			ret += PtrLoopFix(ref ptrType,ref ptrName,true);
			//Dictionary<string,string> usedPointers = new Dictionary<string, string>(Service.PointersUsed);
				
			ret += "for(int i"+depth+"=0; i"+depth+" < len_"+this.name+depth+"; i"+depth+"++) {\n";
				if (this.IsBaseType) {
					if(this.atomicRosType == "string") {	
						ret += "len2_"+this.name+depth+" = (*"+ptrName+");\n";					
						ret += ptrName+"++;\n";
						ret += SetPointerTo("char",ref ptrType, ref ptrName);
						ret += indirection+FirstLetterToUpper(this.name)+ ".Add(new System.String("+ptrName+",0,len2_"+this.name+depth+"));\n";	
						ret += ptrName+" += len2_"+this.name+depth+";\n";
						ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
						//ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
						ret += indirection+FirstLetterToUpper(this.name)+".Add((*"+ptrName+")*1e9ul);\n";
						ret +=ptrName+"++;\n";
						ret += indirection+FirstLetterToUpper(this.name)+"["+indirection+FirstLetterToUpper(this.name)+".Count-1] += (*"+ptrName+");\n";					
						ret +=ptrName+"++;\n";
					
					
					}
					else {
						ret += indirection+FirstLetterToUpper(this.name)+".Add(*"+ptrName+");\n";
						ret += ptrName+"++;\n";	
					}
				}
				
				else {
					ret += indirection+FirstLetterToUpper(this.name)+".Add(new "+this.ServiceRef.CsClassName+"(false));\n";
					ret += this.ServiceRef.FromByte(indirection+FirstLetterToUpper(this.name)+"[i"+depth+"].",ref ptrType,ref ptrName,depth+1);
					ret += this.ServiceRef.PtrLoopFix(ref ptrType,ref ptrName);				
				}
			ret +="}\n";			
			//Service.PointersUsed = usedPointers;
			return ret;
		}
		public string ToByte(string indirection, ref string ptrType,ref string ptrName, int depth) {
			string ret = "";	
			
			if (this.IsBaseType && !this.IsArrayType) {
				if (this.atomicRosType == "string") {					
					ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+".Length;\n";
					ret +=ptrName+"++;\n";
					ret += SetPointerTo("char",ref ptrType, ref ptrName);					
					ret += "byte[] str"+this.name+depth+" = AsciiEncoder.GetBytes("+indirection+FirstLetterToUpper(this.name)+");\n";
					//ret +="char[] str"+this.name+depth+" = "+indirection+FirstLetterToUpper(this.name)+".ToCharArray();\n";
					ret +="for(int i"+this.name+depth+"=0; i"+this.name+depth+"< str"+this.name+depth+".Length; i"+this.name+depth+"++) {\n";
					ret +="\t*"+ptrName+" = (sbyte)str"+this.name+depth+"[i"+this.name+depth+"];\n";
					ret +="\t"+ptrName+"++;\n";
					ret += "}\n";
					//ret += indirection+FirstLetterToUpper(this.name)+".CopyTo(0,"+ptrName+",0,"+indirection+FirstLetterToUpper(this.name)+".Length);\n";
					//ret += ptrName+"+="+indirection+FirstLetterToUpper(this.name)+".Length;\n";
				} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
					ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
					ret += "*"+ptrName+" =(uint)("+indirection+FirstLetterToUpper(this.name)+"/ 1000000000ul);\n";
					ret +=ptrName+"++;\n";
					ret += "*"+ptrName+" =(uint)("+indirection+FirstLetterToUpper(this.name)+"% 1000000000ul);\n";
					ret +=ptrName+"++;\n";
				} /*else if (this.atomicRosType == "bool") {
					ret += SetPointerTo("uint8",ref ptrType, ref ptrName);
					ret += "*"+ptrName+ " = (byte)("+indirection+FirstLetterToUpper(this.name)+"? 1 : 0);\n";					
					ret +=ptrName+"++;\n";
				}*/
				else {
					//simple case
					ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
					ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+";\n";
					ret +=ptrName+"++;\n";
				}
				return ret;
			}
			if (!this.IsArrayType) {
				return this.ServiceRef.ToByte(indirection+FirstLetterToUpper(this.name)+".",ref ptrType,ref ptrName,depth+1);
			}
			if (this.IsFixedSizeArray) {
				if (this.atomicRosType == "string") {
					ret += "byte[] str"+this.name+depth+";\n";
				}				
				for(int i=0; i<this.ArraySize; i++) {
					if (this.IsBaseType) {
						if(this.atomicRosType == "string") {
							ret += SetPointerTo("int32",ref ptrType, ref ptrName);
							ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+"["+i+"].Length;\n";
							ret +=ptrName+"++;\n";
							ret += SetPointerTo("char",ref ptrType, ref ptrName);											
							ret += "str"+this.name+depth+" = AsciiEncoder.GetBytes("+indirection+FirstLetterToUpper(this.name)+"["+i+"]);\n";
							ret +="for(int i"+this.name+depth+"=0; i"+this.name+depth+"< str"+this.name+depth+".Length; i"+this.name+depth+"++) {\n";
							ret +="\t*"+ptrName+" = (sbyte)str"+this.name+depth+"[i"+this.name+depth+"];\n";
							ret +="\t"+ptrName+"++;\n";
							ret += "}\n";
						} else if (this.atomicRosType == "time" || this.atomicRosType == "duration") {
							ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
							ret += "*"+ptrName+" =(uint)("+indirection+FirstLetterToUpper(this.name)+"["+i+"] / 1000000000ul);\n";
							ret +=ptrName+"++;\n";
							ret += "*"+ptrName+" =(uint)("+indirection+FirstLetterToUpper(this.name)+"["+i+"] % 1000000000ul);\n";
							ret +=ptrName+"++;\n";
						}
						else {
							ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
							ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+"["+i+"];\n";
							ret += ptrName+"++;\n";		
						}
					}
					else {
						ret += this.ServiceRef.ToByte(indirection+FirstLetterToUpper(this.name)+"["+i+"].",ref ptrType,ref ptrName,depth+1);
					}
				}
				return ret;
			}
			
			//Array Length:
			ret += SetPointerTo("int32",ref ptrType, ref ptrName);
			ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+".Count;\n";
			ret += ptrName+"++;\n";
			//loop:
			if (this.atomicRosType == "string") {
				ret += "byte[] str"+this.name+depth+";\n";
			}
			ret += PtrLoopFix(ref ptrType,ref ptrName,true);
			//Dictionary<string,string> usedPointers = new Dictionary<string, string>(Service.PointersUsed);
			ret += "for(int i"+depth+"=0; i"+depth+" < "+indirection+FirstLetterToUpper(this.name)+".Count; i"+depth+"++) {\n";
				if (this.IsBaseType) {
					if(this.atomicRosType == "string") {
						ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+"[i"+depth+"].Length;\n";
						ret +=ptrName+"++;\n";
						ret += SetPointerTo("char",ref ptrType, ref ptrName);											
						ret += "str"+this.name+depth+" = AsciiEncoder.GetBytes("+indirection+FirstLetterToUpper(this.name)+"[i"+depth+"]);\n";
						ret +="for(int i"+this.name+depth+"=0; i"+this.name+depth+"< str"+this.name+depth+".Length; i"+this.name+depth+"++) {\n";
						ret +="\t*"+ptrName+" = (sbyte)str"+this.name+depth+"[i"+this.name+depth+"];\n";
						ret +="\t"+ptrName+"++;\n";
						ret += "}\n";
						ret += SetPointerTo("int32",ref ptrType, ref ptrName);
					} else if (this.atomicRosType == "time" || this.atomicRosType=="duration") {
						ret += SetPointerTo("uint32",ref ptrType, ref ptrName);
						ret += "*"+ptrName+" =(uint)("+indirection+FirstLetterToUpper(this.name)+"[i"+depth+"] / 1000000000ul);\n";
						ret +=ptrName+"++;\n";
						ret += "*"+ptrName+" =(uint)("+indirection+FirstLetterToUpper(this.name)+"[i"+depth+"] % 1000000000ul);\n";
						ret +=ptrName+"++;\n";
					} else if (this.atomicRosType == "bool") {
						ret += "*"+ptrName+ " = ("+indirection+FirstLetterToUpper(this.name)+"[i"+depth+"]? 1 : 0);\n";
						ret +=ptrName+"++;\n";
					}
					else {			
						//ret += SetPointerTo(this.atomicRosType,ref ptrType, ref ptrName);
						ret += "*"+ptrName+" = "+indirection+FirstLetterToUpper(this.name)+"[i"+depth+"];\n";
						ret += ptrName+"++;\n";	
					}
				}
				
				else {
					ret += this.ServiceRef.ToByte(indirection+FirstLetterToUpper(this.name)+"[i"+depth+"].",ref ptrType,ref ptrName,depth+1);
					ret += this.ServiceRef.PtrLoopFix(ref ptrType,ref ptrName);				
				}
			ret +="}\n";			
			//Service.PointersUsed = usedPointers;
			return ret;
		}
		public List<string> BaseTypes() {
			List<string> ret = new List<string>();
			if (this.IsArrayType && !this.IsFixedSizeArray) {
				ret.Add("int32");
			}
			if (this.atomicRosType=="string") {
				ret.Add("int32");
				ret.Add("char");
			} else if (this.atomicRosType=="time" || this.atomicRosType=="duration") {
				ret.Add("uint32");
			} else if (this.IsBaseType) ret.Add(this.atomicRosType);
			if(!this.IsBaseType)
			{
				if( this.ServiceRef != null)
					ret.AddRange(this.ServiceRef.BaseTypes());
			}
			
			return ret;
		}
		public string PtrLoopFix(ref string ptrType, ref string ptrName,bool ignoreArray) {
			if ((!ignoreArray && this.IsArrayType) || this.atomicRosType == "string") {
					return SetPointerTo("int32",ref ptrType,ref ptrName);
			}
			if (this.IsBaseType) {
				return SetPointerTo(this.atomicRosType,ref ptrType,ref ptrName);
			}
			if( this.ServiceRef != null )
				return this.ServiceRef.PtrLoopFix(ref ptrType,ref ptrName);			
			else return "";
		}
		protected string SetPointerTo(string reqtype,ref string oldType, ref string oldName) {			
			if (reqtype == oldType) return "";
			string pt;
			if (Service.Mode == Service.GEN_CPP) {
				pt = baseTypeMapping[reqtype].B+"*";
			}
			else pt = baseTypeMapping[reqtype].A+"*";
			string ret = "";
			if (!Service.PointersUsed.ContainsKey(reqtype)) {
					Service.PointersUsed.Add(reqtype,reqtype+"p");
					ret = pt+" ";
			}
			ret += Service.PointersUsed[reqtype]+" = ("+pt+") "+oldName+";\n";
			oldType = reqtype;
			oldName = Service.PointersUsed[reqtype];
			return ret;
		}
		
		public string GetCSType() {
			string type = "";
			if (this.IsBaseType) {
				type = baseTypeMapping[this.atomicRosType].A;				
			} else {
				type = this.rosType.Replace("/",".");				
			}
			if (this.IsFixedSizeArray) {
				if (!this.IsBaseType) return type.Substring(0,type.Length-2)+"[]";
				return type+"[]";
			}
			if (this.IsArrayType) {
				if (!this.IsBaseType) return "List<"+type.Substring(0,type.Length-2)+">";
				return "List<"+type+">";
			}
			return type;
		}
		public string GetCSBaseType() {
			string type = "";
			if (this.IsBaseType) {
				type = baseTypeMapping[this.atomicRosType].A;				
			} else {
				type = this.rosType.Replace("/",".");				
			}
			if (this.IsArrayType && !this.IsBaseType) type=type.Substring(0,type.Length-2);
			return type;
		}
		public string GetCSInitialisation() {
			if (!this.IsArrayType && this.IsBaseType && this.atomicRosType == "string") {
				return FirstLetterToUpper(this.name) + " = System.String.Empty;\n";				
			}
			if (!this.IsArrayType && this.IsBaseType) return "";
			if (this.IsFixedSizeArray)  {
				string ret = FirstLetterToUpper(this.name) + " = new "+GetCSBaseType()+"["+this.ArraySize+"];\n";
				if (!this.IsBaseType) {
					for(int i=0; i<this.ArraySize; i++) {
						ret += FirstLetterToUpper(this.name)+"["+i+"] = new "+GetCSBaseType()+"(true);\n";
					}
				}
				return ret;
			}
			if (this.IsArrayType) return FirstLetterToUpper(this.name) + " = new List<"+GetCSBaseType()+">();\n";
			return FirstLetterToUpper(this.name) + " = new "+GetCSBaseType()+"(true);\n";
		}
		public static void InitBaseTypeMapping() {
			baseTypeMapping = new Dictionary<string,TypeEntry>();
	
			baseTypeMapping.Add("string",new TypeEntry("string","string",-1));
			baseTypeMapping.Add("int8",new TypeEntry("sbyte","int8_t",1));
			baseTypeMapping.Add("uint8",new TypeEntry("byte","uint8_t",1));
			baseTypeMapping.Add("byte",new TypeEntry("byte","uint8_t",1));
			baseTypeMapping.Add("int16",new TypeEntry("short","int16_t",2));
			baseTypeMapping.Add("uint16",new TypeEntry("ushort","uint16_t",2));
			baseTypeMapping.Add("int32",new TypeEntry("int","int32_t",4));
			baseTypeMapping.Add("uint32",new TypeEntry("uint","uint32_t",4));
			baseTypeMapping.Add("int64",new TypeEntry("long","int64_t",8));
			baseTypeMapping.Add("uint64",new TypeEntry("ulong","uint64_t",8));
			baseTypeMapping.Add("bool",new TypeEntry("bool","uint8_t",1));
			baseTypeMapping.Add("float32",new TypeEntry("float","float",4));
			baseTypeMapping.Add("float64",new TypeEntry("double","double",8));
			baseTypeMapping.Add("time",new TypeEntry("ulong","uint32_t",8)); //hackish
			baseTypeMapping.Add("duration",new TypeEntry("ulong","uint32_t",8)); //hackish
			
			baseTypeMapping.Add("char",new TypeEntry("sbyte","char",1));
			
			
			
		}
		public void SetReferences(Dictionary<string,Service> msges) {
			if (this.IsBaseType) return;
			Service m;
			if(msges.TryGetValue(this.atomicRosType,out m)) {			
				this.ServiceRef = m;
			} else {
				throw new Exception("Could not find msg: "+this.atomicRosType);
			}
		}
		public static string FirstLetterToUpper(string val){	
			string retVal = "";
			char tmpLetter = Char.ToUpper(val[0]);
			retVal = tmpLetter + val.Substring(1,val.Length-1);
			
			return retVal;
		}
		public static string FirstLetterToLower(string val){	
			string retVal = "";
			char tmpLetter = Char.ToLower(val[0]);
			retVal = tmpLetter + val.Substring(1,val.Length-1);
				
			return retVal;
		}
		
		
	}
	public class TypeEntry {
		public TypeEntry(string a, string b,int size) {
					this.A = a;
					this.B = b;
					this.Size = size;
				
		}
		public string A {get; set;}
		public string B {get; set;}
		public int Size {get; set;}
	}
	
}
