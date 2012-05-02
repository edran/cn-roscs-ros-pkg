
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

/// <summary>
/// A Ros compatible replacement for <see cref="System.Threading.Timer"/>
/// </summary>
namespace RosCS
{

	public class Timer
	{
		
		int dueTime;
		int interval;
		System.Threading.TimerCallback callback;
		System.Threading.Thread t;
		bool running;
		bool paused = false;
		System.Threading.AutoResetEvent sig;
		public Timer(System.Threading.TimerCallback callback) {
			if(!Node.MainNode.Ok()) { //this will also create the mainNode, if needed
				throw new Exception("Cannot use RosTimer without a working Node!");
			}
			this.sig = new System.Threading.AutoResetEvent(false);
			this.callback = callback;
			this.running = true;
			this.paused = true;
			this.t = new System.Threading.Thread(new System.Threading.ThreadStart(this.Run));
			this.t.Start();
		}
		public Timer(System.Threading.TimerCallback callback,int dueTime,int interval) :this(callback) {
			this.SetActive(dueTime,interval);
		}
		public void SetActive(int dueTime, int interval) {
			this.dueTime = dueTime;
			this.interval = interval;			
			if (this.paused) {
				this.UnPause();
			}
		}
		public void Pause() {
			this.paused = true;
		}
		public void Stop() {
			this.running = false;
			if(t != null) {
				this.t.Abort();
				this.t = null;
			}
		}
		public void UnPause() {
			if (!this.paused) return;
			this.paused = false;
			this.sig.Set();
		}
		public void SetInterval(int interval) {
			this.interval = interval;
		}
		protected void Run() {
			if (this.dueTime > 0) {
				RosSharp.Sleep(this.dueTime);
			}
			while(this.running) {
				if(this.paused) {
					this.sig.WaitOne();
					if(this.dueTime > 0) RosSharp.Sleep(this.dueTime);
				}
				ulong a = RosSharp.Now();
				callback(null);
				ulong b = RosSharp.Now();
				if (a < b) {
					int s = Math.Max(0,this.interval - (int)((b-a)/1000000UL));
					if(s > 0) RosSharp.Sleep(s);
				} else {
					RosSharp.Sleep(this.interval);
				}
			}
		}
		~Timer() {
			if(this.t != null) {
				this.running = false;
				this.UnPause();
				this.t.Abort();
				this.t = null;
			}
			
		}
		
	}
}
