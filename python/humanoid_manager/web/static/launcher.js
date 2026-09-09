/* Robot lifecycle never reloads the browser or stops the web process. */
(() => {
  let state=null,pending=false,shownRobot='';
  const labels={stopped:'已关闭',waiting:'等待 ROS 状态',starting:'正在开启',running:'进程已启动',stopping:'正在关闭',failed:'启动失败 / 进程退出'};
  const current=()=>window.Configurator?.currentRobot();
  function message(text,error=false){setText('#launcherMessage',text);$('#launcherMessage').className=error?'launcher-error':'';}
  function update(){
    const runtime=state?.runtime||{},robot=current(),busy=['waiting','starting','running','stopping'].includes(runtime.phase);
    if(robot&&robot.robot_id!==shownRobot){
      shownRobot=robot.robot_id;
      const plan=state?.profiles[shownRobot];
      $('#launcherTeleop').checked=plan?.start_teleop||false;
      $('#launcherCameras').checked=plan?.start_cameras!==false;
    }
    const unsaved=window.Configurator?.hasUnsavedChanges();
    $('#startManagedRobot').disabled=pending||busy||!runtime.enabled||!robot||unsaved;
    $('#restartManagedRobot').disabled=pending||!runtime.enabled||!robot||unsaved;
    $('#stopManagedRobot').disabled=pending||!busy||!runtime.enabled;
    $('#launcherTeleop').disabled=$('#launcherCameras').disabled=pending||!robot;
    $('#applyRobot').classList.toggle('hidden',!!runtime.enabled);
    if(runtime.enabled)setText('#robotApplyHint','保存配置只生成新版本；点击“重启机器人”后生效，网页不会重启。');
    setText('#launcherPhase',labels[runtime.phase]||'正在读取');
    setText('#launcherTarget',robot?`当前选择：${robot.name} · ${robot.robot_id}${busy?'　｜　运行中：'+runtime.robot_id:''}`:'请先在左侧选择或创建机器人配置。');
    setText('#launcherHint',!runtime.enabled?'当前仅启动网页。请使用 ros2 launch robot_bringup registered_robot.launch.py，才能从这里控制机器人。':unsaved?'有未保存的配置，请先保存；保存不会自动重启或启动机器人。':'保存配置后，重启机器人生效。关闭、重启会处理受控的底层驱动，网页保持运行。');
  }
  async function operation(fn){
    pending=true;update();message('');
    try{await fn();}catch(error){message(error.message,true);}finally{pending=false;update();}
  }
  function start(action){
    const robot=current();
    if(!robot||window.Configurator.hasUnsavedChanges())return message('请先保存机器人配置',true);
    const data={robot_id:robot.robot_id,revision:robot.latest,start_teleop:$('#launcherTeleop').checked,start_cameras:$('#launcherCameras').checked};
    const verb=action==='restart'?'重启':'开启';
    if(!confirm(`${verb} ${robot.name}（${robot.robot_id}）？\n将加载最新保存配置；遥操作：${data.start_teleop?'开启':'关闭'}。\n${action==='restart'?'当前受控机器人及底层驱动会先关闭。':''}请确认工作区域安全，电机可能进入使能或保持状态。`))return;
    operation(async()=>{
      state=await api('/api/launcher/'+action,{method:'POST',body:JSON.stringify(data)});
      await window.Configurator.reloadSelected();
      message(`${verb}命令已完成，加载版本 ${state.runtime.revision}。请查看运行状态确认硬件连接；网页未重启。`);
    });
  }
  $('#startManagedRobot').onclick=()=>start('start');
  $('#restartManagedRobot').onclick=()=>start('restart');
  $('#stopManagedRobot').onclick=()=>operation(async()=>{
    state.runtime=await api('/api/launcher/stop',{method:'POST'});message('机器人及受控底层驱动已关闭，网页保持运行。');
  });
  $('#showLauncherLog').onclick=()=>operation(async()=>{
    const log=await api('/api/launcher/log');setText('#launcherLog',log.text||'尚无启动日志');$('#launcherLog').classList.remove('hidden');
  });
  window.RobotLauncherUI={changed:update,enabled:()=>!!state?.runtime.enabled,status(value){
    if(!state||!value)return;state.runtime=value;update();if(value.error)message(value.error,true);
  }};
  operation(async()=>{state=await api('/api/launcher');shownRobot='';});
})();
