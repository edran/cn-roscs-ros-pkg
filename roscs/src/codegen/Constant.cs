
using System;
using System.Collections.Generic;

namespace CSCodeGen
{


	public class Constant
	{
		string fieldDefinition;
		string rosType;
		string name;
		string val;
		
		public Constant (string def){
			this.fieldDefinition = def;
			string[] strarr = def.Split(' ','\t');
			
			if (strarr.Length!=2) {
				throw new Exception("Unexpected message field format: "+def);
			}
			this.rosType = strarr[0].Trim();
			
			strarr = strarr[1].Split('=');
			if (strarr.Length!=2) {
				throw new Exception("Unexpected message field format: "+def);
			}
			this.name = strarr[0].Trim();
			this.val = strarr[1].Trim();
			Console.WriteLine("Constant Field: {0} - {1} - {2}",this.rosType,this.name,this.val);
			
		}
		public string GetCSDeclaration() {
			if( MessageField.baseTypeMapping[this.rosType].A.Equals("string") )
				return "public const "+MessageField.baseTypeMapping[this.rosType].A+" "+this.name+" = \""+this.val+"\";\n";
			else if( MessageField.baseTypeMapping[this.rosType].A.Equals("float") )
				return "public const "+MessageField.baseTypeMapping[this.rosType].A+" "+this.name+" = "+this.val+"f;\n";
			else
				return "public const "+MessageField.baseTypeMapping[this.rosType].A+" "+this.name+" = "+this.val+";\n";
		}
	}
}
