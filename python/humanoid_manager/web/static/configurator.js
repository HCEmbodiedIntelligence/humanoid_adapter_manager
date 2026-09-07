/* Robot configuration editing never publishes control commands. */
(() => {
  let catalog = null, selected = null, tab = 'driver', dirty = false, settings = null, live = null, loading = false;
  const clone = value => JSON.parse(JSON.stringify(value));
  const tabs = [['driver','驱动参数'],['joints','关节映射'],['model','模型与分组'],['motion','运动参数'],['channels','通道与工具'],['teleop','机械臂遥操作'],['chassis','底盘配置'],['grippers','夹爪配置'],['recording','录制方案']];
  const labels = {
    chassis:'底盘',grippers:'夹爪',command_topic:'控制接口名称',message_type:'底盘消息类型',frame_id:'速度参考坐标系',enable_button:'持续按住的使能按键',
    forward_axis:'前后摇杆轴',lateral_axis:'横移摇杆轴',turn_axis:'转向摇杆轴',invert_forward:'反转前后方向',invert_lateral:'反转横移方向',invert_turn:'反转转向方向',
    max_forward_speed:'前后速度上限（m/s）',max_lateral_speed:'横移速度上限（m/s）',max_turn_speed:'转向速度上限（rad/s）',
    linear_acceleration:'线加速度上限（m/s²）',angular_acceleration:'角加速度上限（rad/s²）',deadband:'输入死区（0–1）',command_timeout:'底盘输入超时（秒）',
    input_axis:'夹爪开合输入',command_type:'夹爪控制接口类型',feedback_type:'反馈消息类型',feedback_topic:'夹爪反馈话题',joint_name:'夹爪关节名',
    position_unit:'位置单位',open_position:'完全打开位置',closed_position:'完全闭合位置',max_speed:'位置变化上限（位置单位/秒）',max_effort:'最大作用力 / 力矩',feedback_timeout:'夹爪反馈超时（秒）',
robot_id:'机器人 ID',buttons_topic:'按钮事件话题',adapter:'管理器关联',
    control_frequency_hz:'控制频率（Hz）', diagnostic_frequency_hz:'诊断频率（Hz）', command_watchdog_ms:'命令超时（ms）',
    platform_joint_state_topic:'关节反馈话题',platform_joint_command_topic:'关节命令话题',diagnostics_topic:'诊断话题',
    plugin_class:'驱动插件类',state_topic:'驱动反馈话题',left_command_topic:'左臂命令话题',right_command_topic:'右臂命令话题',
    left_group:'左臂分组',right_group:'右臂分组',include_gripper:'包含夹爪',left_gripper_joint:'左夹爪关节',right_gripper_joint:'右夹爪关节',
    state_timeout_s:'反馈超时（s）',startup_grace_s:'启动等待时间（s）',velocity_limit_rad_s:'关节速度上限（rad/s）',
    joint_state_endpoint:'关节反馈话题',joint_command_endpoint:'关节命令话题',input_stamp_max_age_s:'输入最大延迟（s）',
    input_stamp_future_tolerance_s:'输入时间容差（s）',feedback_max_age_ms:'反馈最大延迟（ms）',servo_lease_ms:'伺服保持时间（ms）',
    default_move_timeout_s:'运动超时（s）',stable_duration_s:'稳定判定时间（s）',move_j_position_tolerance_rad:'关节位置容差（rad）',
    stopped_velocity_tolerance_rad_s:'停止速度阈值（rad/s）',cartesian_position_tolerance_m:'末端位置容差（m）',cartesian_orientation_tolerance_rad:'末端姿态容差（rad）',
    joint_max_velocity_rad_s:'最大关节速度（rad/s）',joint_max_acceleration_rad_s2:'最大关节加速度（rad/s²）',joint_max_jerk_rad_s3:'最大关节加加速度（rad/s³）',
    cartesian_max_linear_velocity_m_s:'末端最大线速度（m/s）',cartesian_max_linear_acceleration_m_s2:'末端最大线加速度（m/s²）',cartesian_max_linear_jerk_m_s3:'末端最大线加加速度（m/s³）',
    cartesian_max_angular_velocity_rad_s:'末端最大角速度（rad/s）',cartesian_max_angular_acceleration_rad_s2:'末端最大角加速度（rad/s²）',cartesian_max_angular_jerk_rad_s3:'末端最大角加加速度（rad/s³）',
    name:'名称',id:'标识',kind:'运动类型',endpoint:'指令话题',priority:'优先级',group:'关节分组',base_frame:'参考坐标系',tip_frame:'末端坐标系',fk_pose_topic:'实测末端反馈话题',
    parent_frame:'父坐标系',child_frame:'工具坐标系',translation_m:'位置偏移 XYZ（m）',rotation_xyzw:'姿态四元数 XYZW',
    input:'输入设置',control:'控制设置',channels:'通道',mode:'输入方式',bind_host:'监听地址',source_ip:'指定头显 IP（可留空）',pose_port:'位姿接收端口',discovery_port:'发现端口',
    vr_data_topic:'VR 数据话题',publish_vrdata:'发布 VR 原始数据',rate_hz:'输出频率（Hz）',input_timeout:'输入超时（s）',fk_timeout:'实测末端反馈超时（s）',
    enabled_on_start:'启动时启用',resume_on_a:'A 键恢复',emergency_stop_topic:'停止话题',controller:'位姿来源',clutch_controller:'离合手柄',clutch_threshold:'离合阈值（0–1）',
    target_pose_topic:'末端目标话题',tool_frame:'工具坐标系',axis_mapping:'坐标旋转矩阵（3×3）',position_scale:'位移比例',max_displacement:'最大相对位移（m）',
    orientation_enabled:'启用姿态映射',filter_alpha:'滤波系数（0–1）',position_deadband:'位置死区（m）',orientation_deadband:'姿态死区（rad）',workspace:'工作空间（m）',min:'最小 XYZ',max:'最大 XYZ',
    directory:'录制目录',topic:'话题',type:'消息类型',enabled:'启用',max_hz:'限频（Hz，0 不限）',event_max_hz:'网页刷新上限（Hz）',
  };
  const enums = {message_type:['twist','twist_stamped'],command_type:['joint_state','float64','gripper_action'],feedback_type:['joint_state','float64'],position_unit:['m','rad'],input_axis:['trigger','grip'],enable_button:['primary_axis_click','grip_button','trigger_button','primary','secondary','menu','secondary_axis_click'],forward_axis:['primary_y','primary_x','secondary_y','secondary_x','none'],lateral_axis:['none','primary_x','primary_y','secondary_x','secondary_y'],turn_axis:['primary_x','primary_y','secondary_x','secondary_y','none'],kind:['move_j','move_l','move_p','servo_j','servo_p'],controller:['left','right','head'],clutch_controller:['left','right'],mode:['udp','vrdata']};
  const el = (tag, text, cls) => {const node=document.createElement(tag);if(text!==undefined)node.textContent=text;if(cls)node.className=cls;return node;};
  function button(text, action, cls='') {const b=el('button',text,cls);b.type='button';b.onclick=action;return b;}
  const post = (url, value={}) => api(url,{method:'POST',body:JSON.stringify(value)});
  const robotUrl = () => `/api/adapters/robots/${encodeURIComponent(selected.robot_id)}`;
  function notify(message, error=false) {const box=$('#robotOperationResult');box.textContent=message;box.className=`notice ${error?'error':'success'}`;toast(message,error);}
  function markDirty() {dirty=true;setText('#robotDraftState','未保存');setText('#robotOperationResult','');$('#robotOperationResult').classList.add('hidden');}
  function params(key) {return selected.draft.resources[key]?.[key==='driver_params'?'humanoid_driver_runtime':'humanoid_motion_control']?.ros__parameters || {};}
  function choices(key) {
    if(enums[key])return enums[key];
    if(['base_frame','tip_frame','parent_frame','child_frame','tool_frame'].includes(key))return [...(selected.model_info.links||[]),...(selected.draft.resources.tool_config?.tools||[]).map(t=>t.name)];
    if(key==='group')return params('motion_params').joint_group_names||[];
    if(key==='target_pose_topic')return (selected.draft.resources.channel_config?.channels||[]).filter(c=>c.kind==='servo_p').map(c=>c.endpoint);
    return null;
  }
  function scalar(key, value, change, {readonly=false, options=null}={}) {
    const label=el('label',labels[key]||key);
    const opts=options||choices(key);
    let input;
    if(opts){input=el('select');for(const v of [...new Set([value,...opts])]){const option=el('option',v);option.value=v;input.append(option);}input.value=value;}
    else{input=el('input');input.type=typeof value==='boolean'?'checkbox':typeof value==='number'?'number':'text';if(input.type==='checkbox')input.checked=value;else input.value=value??'';if(input.type==='number'){input.step='any';if(key.endsWith('_hz')||key.endsWith('_ms')||key.includes('timeout'))input.min='0';if(key==='filter_alpha'||key==='clutch_threshold'){input.min='0.001';input.max='1';}}}
    input.disabled=readonly;input.setAttribute('aria-label',labels[key]||key);
    input.addEventListener(opts?'change':'input',()=>{
      if(input.type==='number'&&(!input.value.trim()||!Number.isFinite(Number(input.value)))){input.setCustomValidity('请输入有效数字');input.reportValidity();return;}
      input.setCustomValidity('');change(input.type==='checkbox'?input.checked:typeof value==='number'?Number(input.value):input.value);markDirty();
    });
    label.append(input,el('small',key));return label;
  }
  function objectFields(container, object, omit=[]) {
    const grid=el('div',undefined,'parameter-grid');container.append(grid);
    for(const [key,value] of Object.entries(object)){
      if(omit.includes(key))continue;
      if(value!==null&&typeof value==='object')continue;
      grid.append(scalar(key,value,next=>{object[key]=next;},{readonly:key==='schema_version'||key==='plugin_class'||key.endsWith('_config_file')||key==='urdf_file'}));
    }
    for(const [key,value] of Object.entries(object)){
      if(omit.includes(key)||value===null||typeof value!=='object')continue;
      const group=el('fieldset');group.append(el('legend',labels[key]||key));container.append(group);
      if(Array.isArray(value))arrayFields(group,key,value);else objectFields(group,value);
    }
  }
  function arrayFields(container,key,array){
    if(array.every(v=>typeof v==='number')){
      const row=el('div',undefined,'vector-inputs');array.forEach((v,i)=>row.append(scalar(`${key}[${i}]`,v,next=>{array[i]=next;})));container.append(row);return;
    }
    array.forEach((value,i)=>{
      if(value&&typeof value==='object'&&!Array.isArray(value)){
        const field=el('fieldset'),heading=el('div',undefined,'row-heading');heading.append(el('h4',value.name||value.id||`${labels[key]||key} ${i+1}`),button('移除',()=>{array.splice(i,1);markDirty();renderForm();}));field.append(heading);objectFields(field,value);container.append(field);
      }else if(Array.isArray(value)){const row=el('div');arrayFields(row,`${key}[${i}]`,value);container.append(row);}
      else{const row=el('div',undefined,'array-row');row.append(scalar(`${key}[${i}]`,value,next=>{array[i]=next;}),button('移除',()=>{array.splice(i,1);markDirty();renderForm();}));container.append(row);}
    });
    if(!['axis_mapping','translation_m','rotation_xyzw','min','max'].includes(key))container.append(button('添加条目',()=>{array.push(array.length?clone(array[0]):'');markDirty();renderForm();}));
  }
  function table(headers){const wrapper=el('div',undefined,'table-wrap'), t=el('table'),head=el('thead'),row=el('tr');headers.forEach(h=>row.append(el('th',h)));head.append(row);t.append(head);const body=el('tbody');t.append(body);wrapper.append(t);return {wrapper,body};}
  function inputCell(value,change,options){const td=el('td'),control=scalar('',value,change,{options});control.lastChild.remove();td.append(control.querySelector('input,select'));return td;}
  function renderDriver(form){
    const p=params('driver_params');objectFields(form,p,['joint_names','vendor_joint_names','vendor_joint_groups','vendor_to_logical_scales','vendor_to_logical_offsets_rad','plugin_parameters']);
    const field=el('fieldset');field.append(el('legend','厂商通信参数'));form.append(field);
    const entries=p.plugin_parameters||(p.plugin_parameters=[]), grid=el('div',undefined,'parameter-grid');field.append(grid);
    entries.forEach((entry,i)=>{const split=entry.indexOf('=');if(split<0){grid.append(scalar(`参数 ${i+1}`,entry,v=>entries[i]=v));return;}
      const key=entry.slice(0,split),raw=entry.slice(split+1);let value=raw;if(raw==='true'||raw==='false')value=raw==='true';else if(raw!==''&&Number.isFinite(Number(raw)))value=Number(raw);
      grid.append(scalar(key,value,v=>{entries[i]=`${key}=${v}`;}));});
    const add=el('div',undefined,'array-row'),key=el('input'),value=el('input');key.placeholder='参数名';value.placeholder='参数值';key.setAttribute('aria-label','新厂商参数名');value.setAttribute('aria-label','新厂商参数值');
    add.append(key,value,button('添加参数',()=>{if(!key.value.trim()||key.value.includes('='))return toast('请输入有效参数名',true);entries.push(`${key.value.trim()}=${value.value}`);markDirty();renderForm();}));field.append(add);
  }
  function renderJoints(form){
    const p=params('driver_params'),names=p.joint_names||[];
    for(const [key,value] of [['vendor_joint_names',''],['vendor_joint_groups','arm'],['vendor_to_logical_scales',1],['vendor_to_logical_offsets_rad',0]])if(!p[key])p[key]=names.map(()=>value);
    const {wrapper,body}=table(['逻辑关节','厂商关节','厂商分组','比例 / 方向','零位偏移（rad）']);form.append(wrapper);
    names.forEach((name,i)=>{const row=el('tr');row.append(inputCell(name,v=>names[i]=v,(selected.model_info.joints||[]).filter(j=>j.type!=='fixed').map(j=>j.name)));
      for(const key of ['vendor_joint_names','vendor_joint_groups','vendor_to_logical_scales','vendor_to_logical_offsets_rad'])row.append(inputCell(p[key][i],v=>p[key][i]=v));body.append(row);});
    form.append(el('p','比例为负数表示方向反转；修改逻辑关节时需同步模型分组，保存前会交叉校验。','form-help'));
  }
  function renderModel(form){
    const info=selected.model_info;form.append(el('p',`${info.name||'机器人模型'} · ${info.links?.length||0} 个连杆 · ${info.joints?.length||0} 个关节`));
    const upload=el('label','替换 URDF 文件'),input=el('input');input.type='file';input.accept='.urdf,.xml';input.onchange=async()=>{if(input.files[0]){selected.draft.resources.urdf=await input.files[0].text();markDirty();toast('URDF 已载入草稿，保存后检查模型关联');}};upload.append(input);form.append(upload);
    const details=el('details');details.append(el('summary','查看 URDF 关节结构'));const t=table(['关节','类型','父连杆','子连杆']);t.wrapper.classList.add('model-joints');for(const j of info.joints||[]){const row=el('tr');[j.name,j.type,j.parent,j.child].forEach(v=>row.append(el('td',v)));t.body.append(row);}details.append(t.wrapper);form.append(details);
    const p=params('motion_params'),sdk=selected.draft.resources.sdk_config;
    for(const group of p.joint_group_names||[]){
      const field=el('fieldset');field.append(el('legend',`关节组：${group}`));const rows=table(['关节（按运动顺序）','下限（rad）','上限（rad）','操作']);field.append(rows.wrapper);
      const names=p[`groups.${group}`],lower=p[`group_lower_limits.${group}`],upper=p[`group_upper_limits.${group}`];
      const sync=()=>{if(sdk.joint_groups)sdk.joint_groups[group]=clone(names);};
      names.forEach((name,i)=>{const row=el('tr');row.append(inputCell(name,v=>{names[i]=v;sync();},info.joints.filter(j=>j.type!=='fixed').map(j=>j.name)),inputCell(lower[i],v=>lower[i]=v),inputCell(upper[i],v=>upper[i]=v));
        const actions=el('td');actions.append(button('↑',()=>{if(!i)return;for(const array of [names,lower,upper])[array[i-1],array[i]]=[array[i],array[i-1]];sync();markDirty();renderForm();}),button('移除',()=>{for(const array of [names,lower,upper])array.splice(i,1);sync();markDirty();renderForm();}));row.append(actions);rows.body.append(row);});
      field.append(button('添加关节',()=>{names.push(info.joints.find(j=>j.type!=='fixed'&&!names.includes(j.name))?.name||'');lower.push(-1);upper.push(1);sync();markDirty();renderForm();}));form.append(field);
    }
    const add=el('div',undefined,'array-row'),name=el('input');name.placeholder='新关节组名称';name.setAttribute('aria-label','新关节组名称');add.append(name,button('添加关节组',()=>{const key=name.value.trim();if(!/^[A-Za-z_][A-Za-z0-9_]*$/.test(key)||(p.joint_group_names||[]).includes(key))return toast('分组名无效或重复',true);p.joint_group_names.push(key);p[`groups.${key}`]=[];p[`group_lower_limits.${key}`]=[];p[`group_upper_limits.${key}`]=[];sdk.joint_groups[key]=[];markDirty();renderForm();}));form.append(add);
  }
  function renderMotion(form){const p=params('motion_params');objectFields(form,p,Object.keys(p).filter(k=>k.startsWith('groups.')||k.startsWith('group_lower_limits.')||k.startsWith('group_upper_limits.')||k==='joint_group_names'));
    const advanced=el('details');advanced.append(el('summary','高级：SDK 参数（保留原配置单位）'));const fields=el('div');objectFields(fields,selected.draft.resources.sdk_config,['joint_groups']);advanced.append(fields);form.append(advanced);}
  function renderChannels(form){
    const channels=selected.draft.resources.channel_config.channels,tools=selected.draft.resources.tool_config.tools;
    const add=button('新增运动通道',()=>{channels.push({name:`channel_${channels.length+1}`,kind:'servo_p',endpoint:`/teleop/channel_${channels.length+1}`,priority:50,group:params('motion_params').joint_group_names[0],base_frame:selected.model_info.links[0]||'',tip_frame:tools[0]?.name||'',fk_pose_topic:`/teleop/channel_${channels.length+1}/fk_pose`});markDirty();renderForm();});form.append(add);arrayFields(form,'channels',channels);
    const field=el('fieldset');field.append(el('legend','工具坐标'));arrayFields(field,'tools',tools);field.append(button('新增工具',()=>{tools.push({name:`tool_${tools.length+1}`,parent_frame:selected.model_info.links[0]||'',child_frame:`tool_${tools.length+1}`,translation_m:[0,0,0],rotation_xyzw:[0,0,0,1]});markDirty();renderForm();}));form.append(field);
  }
  function renderTeleop(form){
    const receiver=selected.draft.resources.hc_teleop_config;
    if(!receiver){form.append(el('p','可根据现有运动通道创建 hc_teleop_recv 配置，随后在表单中调整。'),button('添加遥操作配置',()=>{const channels=selected.draft.resources.channel_config.channels.filter(x=>x.kind==='servo_p');if(!channels.length){toast('请先在“通道与工具”中添加 servo_p 通道和 FK 反馈话题',true);return;}selected.draft.resources.hc_teleop_config={schema_version:1,adapter:{robot_id:selected.robot_id,buttons_topic:'/hc_teleop_recv/buttons'},input:{mode:'udp',bind_host:'0.0.0.0',source_ip:'',pose_port:5005,discovery_port:5006,vr_data_topic:'/vrdata',publish_vrdata:true},control:{rate_hz:100,input_timeout:0.25,fk_timeout:0.25,enabled_on_start:false,resume_on_a:true,emergency_stop_topic:'/teleop/emergency_stop'},channels:channels.map((c,i)=>({id:(c.name||'channel_'+i).replace(/[^a-zA-Z0-9_]/g,'_'),controller:i===0?'right':'left',clutch_controller:i===0?'right':'left',target_pose_topic:c.endpoint,fk_pose_topic:c.fk_pose_topic||'',base_frame:c.base_frame,tool_frame:c.tip_frame,axis_mapping:[[0,0,-1],[-1,0,0],[0,1,0]],position_scale:0.8,max_displacement:0.8,filter_alpha:0.75,orientation_enabled:true}))};delete selected.draft.resources.teleop_config;markDirty();renderForm();}));return;}
    receiver.adapter??={robot_id:selected.robot_id,buttons_topic:'/hc_teleop_recv/buttons'};
    objectFields(form,receiver,['chassis','grippers']);
    if(selected.draft.resources.hc_teleop_config){const buttons=el('div',undefined,'button-row');
      buttons.append(button('添加通道工作空间',()=>{for(const channel of receiver.channels)channel.workspace??={min:[-1,-1,-1],max:[1,1,1]};markDirty();renderForm();}),button('从运动通道补全关联',()=>{for(const channel of receiver.channels){const motion=selected.draft.resources.channel_config.channels.find(c=>c.endpoint===channel.target_pose_topic);if(motion){channel.base_frame=motion.base_frame;channel.tool_frame=motion.tip_frame;channel.fk_pose_topic=motion.fk_pose_topic||'';}}markDirty();renderForm();}));form.append(buttons);}
  }
  function ensureReceiver(){
    let receiver=selected.draft.resources.hc_teleop_config;
    if(!receiver){receiver={schema_version:1,adapter:{robot_id:selected.robot_id,buttons_topic:'/hc_teleop_recv/buttons'},input:{mode:'udp',bind_host:'0.0.0.0',source_ip:'',pose_port:5005,discovery_port:5006,vr_data_topic:'/vrdata',publish_vrdata:true},control:{rate_hz:100,input_timeout:.25,fk_timeout:.25,enabled_on_start:false,resume_on_a:true,emergency_stop_topic:'/teleop/emergency_stop'},channels:[]};selected.draft.resources.hc_teleop_config=receiver;delete selected.draft.resources.teleop_config;}
    return receiver;
  }
  function addPeripheralRecording(){
    const receiver=selected.draft.resources.hc_teleop_config;if(!receiver)return;
    const plan=selected.draft.recording, entries=[['/hc_teleop_recv/status','std_msgs/msg/String']];
    if(receiver.chassis)entries.push([receiver.chassis.command_topic,receiver.chassis.message_type==='twist_stamped'?'geometry_msgs/msg/TwistStamped':'geometry_msgs/msg/Twist']);
    for(const g of receiver.grippers||[]){entries.push([g.feedback_topic,g.feedback_type==='float64'?'std_msgs/msg/Float64':'sensor_msgs/msg/JointState']);if(g.command_type!=='gripper_action')entries.push([g.command_topic,g.command_type==='float64'?'std_msgs/msg/Float64':'sensor_msgs/msg/JointState']);else entries.push([g.command_topic+'/_action/status','action_msgs/msg/GoalStatusArray']);}
    for(const [topic,type]of entries){const previous=plan.subscriptions.find(x=>x.topic===topic);if(previous&&previous.type!==type){toast(`${topic} 已使用其他消息类型，请先调整录制方案`,true);return;}}
    for(const [topic,type]of entries){const previous=plan.subscriptions.find(x=>x.topic===topic);if(previous){previous.enabled=true;if(!previous.outputs.includes('record'))previous.outputs.push('record');}else plan.subscriptions.push({topic,type,enabled:true,outputs:['record'],max_hz:0,event_max_hz:0});}
    markDirty();toast('底盘和夹爪话题已加入机器人录制方案，保存后可载入使用');
  }
  function peripheralFields(form,object,omit=[]){const grid=el('div',undefined,'parameter-grid');for(const [key,value]of Object.entries(object)){if(!omit.includes(key))grid.append(scalar(key,value,v=>{object[key]=v;},{options:key==='controller'?['left','right']:null}));}form.append(grid);}
  function renderChassis(form){
    const chassis=selected.draft.resources.hc_teleop_config?.chassis;
    form.append(el('p','配置底盘控制接口与摇杆映射。默认关闭；接入底盘控制器后，保存、应用并重启接收端加载。','form-help'));
    if(!chassis){form.append(button('添加底盘配置',()=>{ensureReceiver().chassis={enabled:false,command_topic:'/cmd_vel',message_type:'twist',frame_id:'base_link',controller:'left',enable_button:'primary_axis_click',forward_axis:'primary_y',lateral_axis:'none',turn_axis:'primary_x',invert_forward:false,invert_lateral:false,invert_turn:true,max_forward_speed:.3,max_lateral_speed:.3,max_turn_speed:.6,linear_acceleration:.5,angular_acceleration:1,deadband:.12,rate_hz:30,command_timeout:.25};markDirty();renderForm();},'primary'));return;}
    const notice=el('p','等待接收端状态','form-help');notice.id='chassisRuntimeState';form.append(notice);peripheralFields(form,chassis);
    form.append(el('p','差速底盘将横移轴设为 none；全向底盘可设置横移轴。使能按键松开、输入超时或急停时发送零速度，恢复后需松开按键再按下。','form-help'));
    form.append(button('加入录制方案',addPeripheralRecording));updatePeripheralState(live);
  }
  function renderGrippers(form){
    const grippers=selected.draft.resources.hc_teleop_config?.grippers||[];
    form.append(el('p','按实际夹爪驱动选择接口。位置范围与反馈关节可自行填写；需要新鲜反馈和使能按键才会输出开合目标。','form-help'));
    const tools=el('div',undefined,'button-row');tools.append(button('添加夹爪',()=>{const receiver=ensureReceiver();receiver.grippers??=[];let n=receiver.grippers.length+1;while(receiver.grippers.some(g=>g.id==='gripper_'+n))n++;receiver.grippers.push({id:'gripper_'+n,enabled:false,controller:'right',input_axis:'trigger',enable_button:'grip_button',command_type:'joint_state',command_topic:`/grippers/gripper_${n}/command`,feedback_type:'joint_state',feedback_topic:'/joint_states',joint_name:`gripper_${n}_joint`,position_unit:'m',open_position:.04,closed_position:0,max_speed:.05,max_effort:10,deadband:.01,rate_hz:20,feedback_timeout:.5});markDirty();renderForm();},'primary'));if(grippers.length)tools.append(button('加入录制方案',addPeripheralRecording));form.append(tools);
    if(!grippers.length)form.append(el('p','尚未添加夹爪。可配置单夹爪、双夹爪或多个独立夹爪。'));
    grippers.forEach((g,index)=>{const field=el('fieldset'),heading=el('div',undefined,'row-heading');heading.append(el('h4',g.id),button('移除夹爪',()=>{grippers.splice(index,1);markDirty();renderForm();}));field.append(heading);const state=el('p','等待接收端状态','form-help');state.dataset.gripperState=g.id;field.append(state);peripheralFields(field,g);field.append(el('p','JointState 使用 name / position；Float64 使用单个位置值；GripperCommand 使用 control_msgs Action（需对应消息包与 Action 服务）。反馈支持 JointState 或 Float64。','form-help'));form.append(field);});updatePeripheralState(live);
  }
  function updatePeripheralState(data){const event=data?.platform?.teleop;const identity=event?.data?.configuration?.robot_id;const running=event?.fresh&&(!identity||identity===selected?.robot_id);const base=event?.data?.chassis;setText('#chassisRuntimeState',running&&base?`运行状态：${base.state} · ${base.reason||''} · ${base.command_topic||''}`:'当前配置没有新鲜的接收端状态，保存后由接收端加载');for(const node of $$('[data-gripper-state]')){const g=event?.data?.grippers?.[node.dataset.gripperState];node.textContent=running&&g?`运行状态：${g.state} · ${g.reason||''} · 反馈 ${g.feedback??'—'} / 目标 ${g.target??'—'} ${g.position_unit||''}`:'当前夹爪接口尚未接入';}}
  function renderRecording(form){
    const recording=selected.draft.recording;form.append(scalar('directory',recording.directory,v=>recording.directory=v));
    const t=table(['录制','话题','消息类型','限频（Hz）','操作']);
    recording.subscriptions.forEach((rule,i)=>{const row=el('tr');row.append(inputCell(rule.enabled!==false,v=>rule.enabled=v),inputCell(rule.topic,v=>rule.topic=v),inputCell(rule.type,v=>rule.type=v,STANDARD_ROS_TYPES),inputCell(rule.max_hz||0,v=>rule.max_hz=v));const action=el('td');action.append(button('移除',()=>{recording.subscriptions.splice(i,1);markDirty();renderForm();}));row.append(action);t.body.append(row);});form.append(t.wrapper);
    const choice=el('select');choice.setAttribute('aria-label','从 ROS Graph 添加录制话题');choice.append(el('option','从当前 ROS Graph 选择话题'));for(const item of discoveredTopics){const option=el('option',item.topic);option.value=item.topic;choice.append(option);}choice.onchange=()=>{const item=discoveredTopics.find(t=>t.topic===choice.value);if(item&&!recording.subscriptions.some(t=>t.topic===item.topic)){recording.subscriptions.push({topic:item.topic,type:item.types[0],enabled:true,outputs:['record'],max_hz:0,event_max_hz:0});markDirty();renderForm();}};form.append(choice,button('添加自定义话题',()=>{recording.subscriptions.push({topic:'/custom/topic',type:'std_msgs/msg/String',enabled:true,outputs:['record'],max_hz:0,event_max_hz:0});markDirty();renderForm();}));
    form.append(el('p','此处保存机器人专属录制方案。在“话题录制”页载入并应用后，下一次录制使用该方案。','form-help'));
  }
  const renderers={driver:renderDriver,joints:renderJoints,model:renderModel,motion:renderMotion,channels:renderChannels,teleop:renderTeleop,chassis:renderChassis,grippers:renderGrippers,recording:renderRecording};
  function renderForm(){if(!selected)return;const form=$('#robotForm');form.replaceChildren(el('h3',tabs.find(t=>t[0]===tab)[1]));renderers[tab](form);for(const t of $$('table',form)){const headers=$$('th',t).map(h=>h.textContent);$$('tbody tr',t).forEach((row,index)=>$$('td',row).forEach((cell,column)=>{const control=cell.querySelector('input,select');if(control&&!control.getAttribute('aria-label'))control.setAttribute('aria-label',`${headers[column]} · 第 ${index+1} 行`);}));}}
  function renderEditor(){
    $('#robotEmpty').classList.toggle('hidden',!!selected);$('#robotEditor').classList.toggle('hidden',!selected);if(!selected)return;
    $('#managedRobotName').value=selected.draft.name;setText('#robotDraftState',selected.diff.length?'草稿有修改':'已保存');setText('#robotSavedRevision',selected.latest);setText('#robotDeployedRevision',selected.deployed?.revision|| (selected.deployed?'外部部署':'尚未部署'));
    const tablist=$('#robotTabs');tablist.replaceChildren();for(const [id,title] of tabs){const b=button(title,()=>{tab=id;renderEditor();});b.setAttribute('role','tab');b.setAttribute('aria-selected',String(tab===id));tablist.append(b);}renderForm();
    const diff=$('#robotDiff');diff.replaceChildren();setText('#robotDiffCount',`${selected.diff.length} 项`);const t=table(['字段','已保存','当前草稿']);for(const change of selected.diff){const row=el('tr');row.append(el('td',change.path));for(const key of ['before','after']){const cell=el('td');cell.append(el('pre',JSON.stringify(change[key],null,2)));row.append(cell);}t.body.append(row);}diff.append(t.wrapper);
    const history=$('#robotHistory');history.replaceChildren();for(const version of selected.history){const row=el('div');row.append(el('code',version.revision),el('time',new Date(version.created_at).toLocaleString()),button('恢复到草稿',()=>operation(async()=>{if(dirty&&!confirm('有未保存修改，恢复历史版本将替换当前草稿，是否继续？'))return;selected=await post(`${robotUrl()}/restore`,{revision:version.revision,etag:selected.etag});dirty=false;renderEditor();notify('历史版本已放入草稿，校验并保存后可应用');})));const link=el('a','导出配置包');link.href=`${robotUrl()}/export?revision=${encodeURIComponent(version.revision)}`;row.append(link);history.append(row);}
  }
  async function selectRobot(id){if(dirty&&!confirm('当前表单有未保存修改，切换机器人将放弃这些修改，是否继续？'))return;selected=await api(`/api/adapters/robots/${encodeURIComponent(id)}`);dirty=false;renderCatalog();renderEditor();}
  function renderCatalog(){
    const list=$('#managedRobots');list.replaceChildren();if(!catalog)return;
    for(const robot of catalog.workspaces){const b=button('',()=>operation(()=>selectRobot(robot.robot_id)),selected?.robot_id===robot.robot_id?'selected':'');b.append(el('strong',robot.name),el('small',robot.robot_id));list.append(b);}if(!catalog.workspaces.length)list.append(el('p','尚无配置。可导入插件，或复制已部署机器人。'));
    const select=$('#recordingRobotSelect'),previous=select.value;select.replaceChildren(el('option','选择机器人方案'));select.firstChild.value='';for(const robot of catalog.workspaces){const option=el('option',robot.name);option.value=robot.robot_id;select.append(option);}select.value=previous;
  }
  async function refresh(){catalog=await api('/api/adapters');$('#managerError').classList.add('hidden');renderCatalog();if(!selected&&catalog.workspaces.length)await selectRobot(catalog.workspaces[0].robot_id);}
  async function open(){if(catalog||loading)return;loading=true;try{await refresh();}catch(error){const box=$('#managerError');box.textContent=error.message;box.className='notice error';}finally{loading=false;}}
  async function operation(fn){const buttons=$$('#robotEditor button');buttons.forEach(b=>b.disabled=true);try{return await fn();}catch(error){notify(error.message,true);}finally{buttons.forEach(b=>b.disabled=false);}}
  async function persistDraft(){if(!selected)return;const invalid=$('#robotForm :invalid');if(invalid){invalid.reportValidity();throw new Error('请修正表单中的无效参数');}selected=await post(`${robotUrl()}/draft`,{document:selected.draft,etag:selected.etag});dirty=false;renderEditor();}
  function settingsLoaded(state){settings=state;setText('#settingsState',state.external_change?'外部修改，需重新读取':state.pending?'已保存，待应用':'已应用');$('#settingsState').classList.toggle('stale',state.pending||state.external_change);
    setText('#recordingConfigHint',state.pending?'配置已保存，运行中仍使用原配置；停止录制后点击“应用已保存配置”。':'运行中使用已保存配置。修改后先保存，再应用。');
    const history=$('#settingsHistory');history.replaceChildren();for(const version of state.history){const row=el('div');row.append(el('time',new Date(version.created_at*1000).toLocaleString()),button('恢复到表单',async()=>{try{const next=await post('/api/settings/restore',{etag:settings.etag,revision:version.revision});settingsLoaded(next);config=clone(next.draft);renderConfig();toast('历史配置已放入表单，保存并应用后生效');}catch(e){toast(e.message,true);}}));history.append(row);}
    const managed=state.active.adapter_manager?.enabled;document.body.classList.toggle('platform-mode',!!managed);$('#platformStatusPanel').classList.toggle('hidden',!managed);

  }
  async function saveSettings(){try{if(!settings)settings=await api('/api/settings');const draft=await post('/api/settings/draft',{config:collectConfig(),etag:settings.etag});settingsLoaded(draft);const saved=await post('/api/settings/save',{etag:settings.etag});settingsLoaded(saved);config=clone(saved.draft);renderConfig();toast('配置已保存；点击应用后生效');}catch(error){toast(error.message,true);}}
  async function applySettings(){try{const state=await post('/api/settings/apply',{etag:settings.etag});settingsLoaded(state);toast(state.server_restart_required?'配置已保存，服务地址、ROS 域或启动项变化需要重启网页服务':'已应用保存的配置');}catch(error){toast(error.message,true);}}
  async function loadRecordingPlan(){try{const id=$('#recordingRobotSelect').value;if(!id)throw new Error('请选择机器人方案');const robot=await api(`/api/adapters/robots/${encodeURIComponent(id)}`);const plan=robot.saved.recording;
    collectConfig();config.ros.recording.directory=plan.directory;for(const sub of config.ros.subscriptions)sub.outputs=(sub.outputs||[]).filter(x=>x!=='record');for(const rule of plan.subscriptions){const existing=config.ros.subscriptions.find(x=>x.topic===rule.topic);if(existing){const websocket=existing.outputs?.includes('websocket');Object.assign(existing,clone(rule));if(websocket&&!existing.outputs.includes('websocket'))existing.outputs.push('websocket');}else config.ros.subscriptions.push(clone(rule));}renderConfig();setText('#settingsState','表单已修改');toast('机器人录制方案已载入表单，保存并应用后生效');
  }catch(e){toast(e.message,true);}}
  async function storeRecordingPlan(){try{const id=$('#recordingRobotSelect').value;if(!id)throw new Error('请选择机器人方案');const robot=await api(`/api/adapters/robots/${encodeURIComponent(id)}`);const current=collectConfig();robot.draft.recording={directory:current.ros.recording.directory,subscriptions:clone(current.ros.subscriptions.filter(x=>x.outputs?.includes('record')))};const draft=await post(`/api/adapters/robots/${encodeURIComponent(id)}/draft`,{document:robot.draft,etag:robot.etag});const saved=await post(`/api/adapters/robots/${encodeURIComponent(id)}/validate`,{etag:draft.etag,save:true});if(selected?.robot_id===id&&!dirty){selected=saved;renderEditor();}toast('录制方案已保存到机器人配置版本');}catch(e){toast(e.message,true);}}
  function status(data){updatePeripheralState(data);live=data;const platform=data.platform||{},current=platform.configuration;const managed=platform.enabled;
    setText('#runningRobot',managed?(current?.fresh?current.data.name||current.data.robot_id:'未检测到运行机器人'):(config?.robot_profiles?.active||'HC 遥操作'));
    setText('#runningRevision',current?.fresh?(current.data.revision||'外部部署（无版本标识）'):'未知');$('#runningRevision').classList.toggle('stale',!current?.fresh);
    const recording=data.recording||{};setText('#sessionRecording',data.capture?.active?`对齐采集 · ${data.capture.counters?.rows||0} 行`:recording.recording?`${recording.duration_seconds||0}s · ${recording.messages||0} 条`:'未录制');
    setText('#recordingStatistics',`文件大小：${((recording.size_bytes||0)/1048576).toFixed(2)} MB · 已写入：${recording.messages||0} 条 · 丢弃：${recording.dropped||0} 条${recording.error?' · 写入错误：'+recording.error:''}`);
    if(settings){for(const id of ['applyConfig','applyRecording'])$('#'+id).disabled=!!recording.recording||!!data.replay?.is_active;}
    const panel=$('#platformDetails');panel.replaceChildren();if(managed){
      for(const [key,title] of [['configuration','配置与进程'],['diagnostics','驱动连接'],['teleop','遥操作']]){const block=el('div'),item=platform[key];block.append(el('strong',title));let content='尚未收到状态';if(item){if(key==='diagnostics'){content=(item.data.status||[]).filter(s=>s.name?.includes('driver')).map(s=>`${s.name}: ${s.message}\n${(s.values||[]).filter(v=>['connected','active','watchdog_stopped','last_stop_reason'].includes(v.key)).map(v=>`${v.key}: ${v.value}`).join(' · ')}`).join('\n')||'尚未收到驱动诊断';}else if(key==='configuration'){content=`${item.data.robot_id||'未知'} · ${item.data.state||'未知'}\n${item.data.missing_nodes?.length?'等待节点：'+item.data.missing_nodes.join(', '):'启动的节点已在 ROS Graph 中出现'}`;}else{const t=item.data;content=`${t.enabled?'遥操作已启用':'遥操作未启用'} · 已接收 ${t.received_packets||0} 包 · 拒绝 ${t.rejected_packets||0} 包\n${Object.entries(t.channels||{}).map(([id,c])=>`${id}: ${c.state} · ${c.reason||''}`).join('\n')}\n接收端配置：${t.configuration?.robot_id||'未指定机器人'} · ${t.configuration?.sha256?.slice(0,12)||'未知'}`;}content+=`\n${item.fresh?'最近更新':'状态过期'}：${item.age}s 前`;}block.append(el('p',content));panel.append(block);}
      const control=$('#controlSourceState')?.closest('article');if(control)control.classList.add('hidden');
    }
    if(selected){const ros=data.ros||{},nodes=ros.discovered_nodes||[];const busy=nodes.some(n=>['humanoid_driver_runtime','humanoid_motion_control','hc_teleop_recv','humanoid_configuration_status'].includes(n.name));const enabled=ros.state==='running'&&ros.graph_age<=5&&!busy&&!recording.recording&&!data.replay?.is_active;$('#applyRobot').disabled=!enabled;
      setText('#robotApplyHint',recording.recording?'正在录制，可保存草稿和版本；停止录制后再应用。':busy?'机器人正在运行。停止对应启动进程后，可应用已保存版本。':enabled?'可应用已保存版本。应用完成后，下一次启动机器人时加载。':'等待新鲜 ROS 状态以确认机器人已停止；当前可编辑和保存配置。');}
  }
  function disconnected(){setText('#runningRobot','网页服务连接中断');setText('#runningRevision','未知');setText('#sessionRecording','状态未知');$('#applyRobot').disabled=true;}
  function setupCreateChoices(){if(!catalog)return;const mode=$('#managedCreateSource').value;$('#managedPluginChoices').classList.toggle('hidden',mode!=='plugins');$('#managedSourceRobotLabel').classList.toggle('hidden',mode==='plugins');const select=$('#managedSourceRobot');select.replaceChildren();const source=mode==='workspace'?catalog.workspaces.map(x=>[x.robot_id,x.name]):Object.entries(catalog.robots).map(([id,x])=>[id,x.name]);for(const [id,name] of source){const opt=el('option',`${name} · ${id}`);opt.value=id;select.append(opt);}for(const [id,key] of [['managedDriverChoice','hardware_drivers'],['managedModelChoice','robot_models']]){const node=$('#'+id);node.replaceChildren();for(const [keyId,value] of Object.entries(catalog[key])){const opt=el('option',`${value.name} · ${keyId}`);opt.value=keyId;node.append(opt);}}}
  async function showCreate(){try{await refresh();setupCreateChoices();$('#managedCreateError').textContent='';$('#managedCreateDialog').showModal();}catch(e){toast(e.message,true);}}
  window.Configurator={open,status,disconnected,settingsLoaded,saveSettings};
  $('#managedRobotName').oninput=event=>{selected.draft.name=event.target.value;markDirty();};
  $('#refreshRobots').onclick=()=>operation(async()=>{if(dirty&&!confirm('刷新会放弃当前表单的未保存修改，是否继续？'))return;const id=selected?.robot_id;dirty=false;await refresh();if(id)await selectRobot(id);});
  $('#createRobot').onclick=showCreate;$('#emptyCreateRobot').onclick=showCreate;$('#managedCreateSource').onchange=setupCreateChoices;
  $('#managedCreateForm').onsubmit=async event=>{event.preventDefault();const b=event.submitter;b.disabled=true;try{const data=Object.fromEntries(new FormData(event.target));const mode=$('#managedCreateSource').value;if(mode==='deployed')data.source_robot=$('#managedSourceRobot').value;else if(mode==='workspace')data.source_workspace=$('#managedSourceRobot').value;else{data.driver_id=$('#managedDriverChoice').value;data.model_id=$('#managedModelChoice').value;}selected=await post('/api/adapters/robots',data);dirty=false;$('#managedCreateDialog').close();await refresh();renderEditor();location.hash='robots';}catch(e){$('#managedCreateError').textContent=e.message;}finally{b.disabled=false;}};
  $('#importRobotBundle').onclick=()=>{$('#managedImportError').textContent='';$('#managedImportDialog').showModal();};
  $('#managedImportKind').onchange=()=>$('#managedImportIdentity').classList.toggle('hidden',$('#managedImportKind').value!=='workspace');
  $('#managedImportForm').onsubmit=async event=>{event.preventDefault();const b=event.submitter;b.disabled=true;try{const data=new FormData(event.target);if(data.get('kind')==='workspace'&&(!data.get('robot_id')||!data.get('name')))throw new Error('请填写新配置 ID 和名称');const result=await api('/api/adapters/import',{method:'POST',body:data});$('#managedImportDialog').close();await refresh();if(result.robot_id)await selectRobot(result.robot_id);toast('配置包已校验并导入');}catch(e){$('#managedImportError').textContent=e.message;}finally{b.disabled=false;}};
  $$('[data-close-dialog]').forEach(b=>b.onclick=()=>$('#'+b.dataset.closeDialog).close());
  $('#saveRobotDraft').onclick=()=>operation(async()=>{await persistDraft();notify('草稿已保存，运行配置未改变');});
  $('#validateRobot').onclick=()=>operation(async()=>{await persistDraft();const result=await post(`${robotUrl()}/validate`,{etag:selected.etag});notify(result.message);});
  $('#saveRobotVersion').onclick=()=>operation(async()=>{await persistDraft();selected=await post(`${robotUrl()}/validate`,{etag:selected.etag,save:true});renderEditor();await refresh();notify('配置已校验并保存为新版本');});
  $('#applyRobot').onclick=()=>operation(async()=>{const result=await post(`${robotUrl()}/apply`,{etag:selected.etag,revision:selected.latest});selected=await api(robotUrl());renderEditor();notify(`已部署 ${result.deployed.revision}，下一次启动机器人时加载`);});
  $('#applyConfig').onclick=applySettings;$('#applyRecording').onclick=applySettings;$('#loadRobotRecording').onclick=loadRecordingPlan;$('#storeRobotRecording').onclick=storeRecordingPlan;
  $('#reloadSettings').onclick=async()=>{try{const next=await api('/api/settings');settingsLoaded(next);config=clone(next.external_change?next.saved:next.draft);renderConfig();toast('已重新读取配置');}catch(e){toast(e.message,true);}};
  $$('[data-record-preset]').forEach(b=>b.onclick=async()=>{await loadTopics();const kind=b.dataset.recordPreset;let count=0;for(const item of discoveredTopics){const type=item.types?.[0]||'';if((kind==='joints'&&type==='sensor_msgs/msg/JointState')||(kind==='cameras'&&type==='sensor_msgs/msg/CompressedImage')||(kind==='teleop'&&(/^\/teleop\/|^\/hc_teleop_recv\/|^\/vrdata$/.test(item.topic)))){setTopicRecording(item.topic,type,true);count++;}}renderConfig();toast(`已勾选 ${count} 个话题，保存并应用后生效`);});
  window.addEventListener('beforeunload',event=>{if(dirty){event.preventDefault();event.returnValue='';}});
  if(location.hash==='#robots')open();
  // The recording plan selector is useful even when the editor is never opened.
  api('/api/settings').then(state=>{if(state.active.adapter_manager?.enabled)open();}).catch(()=>{});
})();
