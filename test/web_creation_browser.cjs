/* Optional test: use installed Playwright or HUMANOID_TEST_PLAYWRIGHT_MODULE.
 * HUMANOID_TEST_BROWSER_EXECUTABLE can select an existing Chromium binary.
 * Only the temporary, ROS-disabled server supplied by test_web_creation.py is used.
 */
const assert = require('node:assert/strict');
let chromium;
try {
  ({chromium} = require(process.env.HUMANOID_TEST_PLAYWRIGHT_MODULE || 'playwright'));
} catch (error) {
  if (process.env.HUMANOID_TEST_PLAYWRIGHT_MODULE) throw error;
  console.log('SKIP: install Playwright to run browser creation regression');
  process.exit(77);
}
const base = process.argv[2], expected = JSON.parse(process.argv[3]);
assert.equal(new URL(base).hostname, '127.0.0.1', 'Only a temporary loopback test server is allowed');

(async () => {
  let browser;
  try {
    browser = await chromium.launch({headless: true,
      executablePath: process.env.HUMANOID_TEST_BROWSER_EXECUTABLE || undefined});
  } catch (error) {
    if (!process.env.HUMANOID_TEST_BROWSER_EXECUTABLE && error.message.includes("Executable doesn't exist")) {
      console.log('SKIP: install a Playwright Chromium browser');
      process.exitCode = 77;
      return;
    }
    throw error;
  }
  const page = await browser.newPage();
  const pageErrors = [];
  try {
    page.setDefaultTimeout(15000);
    const submissions = [], mutations = [];
    page.on('pageerror', error => pageErrors.push(error.message));
    page.on('console', message => {
      if (message.type() === 'error') console.error('Browser console:', message.text());
    });
    page.on('request', request => {
      if (request.method() === 'POST') {
        mutations.push(new URL(request.url()).pathname);
        if (new URL(request.url()).pathname === '/api/adapters/robots') submissions.push(request.postDataJSON());
      }
    });
    await page.goto(base + 'dashboard/#robots');
    const form = page.locator('#managedCreateForm');
    const submit = form.locator('button[type="submit"]');
    const error = page.locator('#managedCreateError');
    async function open(id='openarmx_01') {
      await page.locator('#createRobot').click();
      await page.waitForFunction(() => document.querySelector('#managedCreateDialog').open, null, {timeout: 5000});
      await form.locator('[name="robot_id"]').fill(id);
      await form.locator('[name="name"]').fill('OpenArmX 双臂');
    }
    async function errorContains(text) {
      await page.waitForFunction(text => document.querySelector('#managedCreateError').textContent.includes(text), text);
    }
    await open('openarmx_01\u200b');
    await submit.click();
    await errorContains('配置 ID');
    assert.equal(submissions.length, 0);
    await form.locator('[name="robot_id"]').fill('openarmx_01');
    await page.locator('#managedDriverChoice').evaluate(node => node.replaceChildren());
    await submit.click();
    await errorContains('机械臂驱动插件');
    assert.equal(submissions.length, 0);
    await form.locator('[data-close-dialog]').first().click();
    await open();
    await page.locator('#managedModelChoice').evaluate(node => node.replaceChildren());
    await submit.click();
    await errorContains('模型插件');
    assert.equal(submissions.length, 0);
    await form.locator('[data-close-dialog]').first().click();
    await open(' \u00a0openarmx_01 ');
    await page.locator('#managedDriverChoice').selectOption(expected.driver_id);
    await page.locator('#managedModelChoice').selectOption(expected.model_id);
    await page.locator('#managedGripperChoice').selectOption(expected.gripper_id);
    await page.locator('#managedCreateSource').selectOption('workspace');
    await submit.click();
    await errorContains('要复制的机器人配置');
    assert.equal(submissions.length, 0);
    await page.locator('#managedCreateSource').selectOption('plugins');
    assert.equal(await page.locator('#managedGripperChoice').inputValue(), expected.gripper_id);
    const createdResponse = page.waitForResponse(r => r.request().method() === 'POST' && r.url().endsWith('/api/adapters/robots'));
    await submit.click();
    const created = await createdResponse;
    assert.equal(created.status(), 201, await created.text());
    await page.waitForFunction(() => !document.querySelector('#managedCreateDialog').open);
    assert.deepEqual(submissions[0], expected);
    assert.equal(await error.textContent(), '');

    // Exercise the actual instance editor without starting any device.
    await page.getByRole('tab', {name:'夹爪驱动', exact:true}).click();
    await page.getByRole('button', {name:'转换为独立实例', exact:true}).click();
    assert.equal(await page.getByLabel('instance_id', {exact:true}).inputValue(), 'default');
    const instanceSave = page.waitForResponse(r=>r.request().method()==='POST'&&r.url().endsWith('/save'));
    await page.locator('#saveRobotConfig').click();
    const instanceResponse = await instanceSave;
    assert.equal(instanceResponse.status(),200,await instanceResponse.text());
    const instanceDocument = (await instanceResponse.json()).saved;
    assert.equal(instanceDocument.gripper_driver,null);
    assert.equal(instanceDocument.gripper_instances.length,1);
    assert.equal(instanceDocument.gripper_instances[0].instance_id,'default');

    // Camera count comes from the editable list. Exercise four independent entries.
    await page.getByRole('tab', {name:'相机配置', exact:true}).click();
    const cameraNames=['front','left','right','rear'];
    for(const [index,name] of cameraNames.entries()){
      await page.getByRole('button',{name:'添加 RealSense 相机',exact:true}).click();
      const field=page.locator('#robotForm > fieldset').last();
      await field.getByLabel('标识',{exact:true}).fill(name);
      await field.getByLabel('相机型号',{exact:true}).fill(index===3?'d455':'d405');
      await field.getByLabel('设备序列号',{exact:true}).fill(`0000000000${index}`);
      await field.getByLabel('彩色图像话题',{exact:true}).fill(`/${name}/rgb`);
      await field.getByLabel('标准化 RGB-D 话题',{exact:true}).fill(`/${name}/rgbd`);
    }
    await page.getByRole('button',{name:'添加 RealSense 相机',exact:true}).click();
    assert.equal(await page.locator('[data-camera-test]').count(),5);
    await page.locator('#robotForm > fieldset').last().getByRole('button',{name:'移除相机',exact:true}).click();
    assert.equal(await page.locator('[data-camera-test]').count(),4);
    const cameraSave=page.waitForResponse(r=>r.request().method()==='POST'&&r.url().endsWith('/save'));
    await page.locator('#saveRobotConfig').click();
    const cameraResponse=await cameraSave;
    assert.equal(cameraResponse.status(),200,await cameraResponse.text());
    const cameraDocument=(await cameraResponse.json()).saved;
    assert.deepEqual(cameraDocument.cameras.map(c=>c.id),cameraNames);
    assert.equal(cameraDocument.cameras[3].serial_no,'00000000003');
    assert.equal(cameraDocument.cameras[3].rgb_topic,'/rear/rgb');
    const testPhoto=await page.evaluate(()=>{
      const canvas=document.createElement('canvas');canvas.width=2;canvas.height=2;
      const ctx=canvas.getContext('2d');ctx.fillStyle='#c04020';ctx.fillRect(0,0,2,2);
      return canvas.toDataURL('image/jpeg');
    });
    let failedCamera=null;
    const captured=[];
    await page.route('**/cameras/*/snapshot?*',async route=>{
      const name=new URL(route.request().url()).pathname.split('/').at(-2);captured.push(name);
      if(name===failedCamera)return route.fulfill({status:409,body:'未收到图像，请检查序列号'});
      const camera=cameraDocument.cameras.find(c=>c.id===name);
      return route.fulfill({json:{ok:true,camera_id:name,device_type:camera.device_type,
        serial_no:camera.serial_no,topic:camera.rgb_topic,width:640,height:480,
        image_data_url:testPhoto,received_at:Date.now()/1000}});
    });
    for(const name of cameraNames){
      const test=page.locator(`[data-camera-test="${name}"]`);
      await test.getByRole('button',{name:'测试拍照',exact:true}).click();
      await test.locator('img').waitFor({state:'visible'});
      await page.waitForFunction(name=>document.querySelector(`[data-camera-test="${name}"] img`).naturalWidth>0,name);
      assert.match(await test.getByRole('status').textContent(),new RegExp(`/${name}/rgb`));
    }
    assert.deepEqual(captured,cameraNames);
    failedCamera='rear';
    const rearTest=page.locator('[data-camera-test="rear"]');
    await rearTest.getByRole('button',{name:'测试拍照',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('[data-camera-test="rear"] [role="status"]').textContent.includes('拍照失败'));
    assert.equal(await rearTest.locator('img').isVisible(),false);
    assert.match(await rearTest.getByRole('status').textContent(),/未收到图像/);

    // Reloading proves the result is persisted, not only a modal-local value.
    await page.reload();
    const saved = await (await page.request.get(base + 'api/adapters/robots/openarmx_01')).json();
    assert.equal(saved.robot_id, 'openarmx_01');
    assert.deepEqual(saved.saved.cameras.map(c=>c.id),cameraNames);

    // UI motion checks use a simulated deployed identity and mocked hardware endpoint.
    await page.route('**/api/adapters/robots/openarmx_01',async route=>{
      const response=await route.fetch(),detail=await response.json();
      detail.deployed={revision:detail.latest};await route.fulfill({response,json:detail});
    });
    const gripperTests=[];
    await page.route('**/grippers/*/test',async route=>{
      const body=route.request().postDataJSON();gripperTests.push(body.target);
      await route.fulfill({json:{ok:true,final_position:body.target==='open'?.07:.01}});
    });
    page.on('dialog',dialog=>dialog.accept());
    await page.reload();
    await page.getByRole('tab',{name:'夹爪驱动',exact:true}).click();
    const gripperTest=page.locator('[data-gripper-test]').first();
    const gripperName=await gripperTest.getAttribute('data-gripper-test');
    for(const label of ['测试打开','测试闭合']){
      await gripperTest.getByRole('button',{name:label,exact:true}).click();
      await page.waitForFunction(label=>document.querySelector('[data-gripper-test] [role="status"]').textContent.includes(label+'完成'),label);
    }
    assert.deepEqual(gripperTests,['open','close']);
    await open();
    await submit.click();
    await errorContains('配置 ID 已存在');
    const unchanged = await (await page.request.get(base + 'api/adapters/robots/openarmx_01')).json();
    assert.equal(unchanged.latest, saved.latest);

    await form.locator('[name="robot_id"]').fill('openarmx_copy');
    await page.locator('#managedCreateSource').selectOption('workspace');
    await page.locator('#managedSourceRobot').selectOption('openarmx_01');
    const copiedResponse = page.waitForResponse(r => r.request().method() === 'POST' && r.url().endsWith('/api/adapters/robots'));
    await submit.click();
    const copied = await copiedResponse;
    assert.equal(copied.status(), 201, await copied.text());
    assert.deepEqual(submissions.at(-1), {robot_id: 'openarmx_copy', name: 'OpenArmX 双臂', source_workspace: 'openarmx_01'});
    await page.waitForFunction(() => !document.querySelector('#managedCreateDialog').open);
    await open('openarmx_no_gripper');
    await page.locator('#managedCreateSource').selectOption('plugins');
    await page.locator('#managedGripperChoice').selectOption('');
    const optionalResponse = page.waitForResponse(r => r.request().method() === 'POST' && r.url().endsWith('/api/adapters/robots'));
    await submit.click();
    const optional = await optionalResponse;
    if (saved.draft.resources.hc_teleop_config?.grippers?.length) {
      // A model that explicitly maps grippers must not bypass composition
      // validation when the user selects "no gripper" in the generic form.
      assert.equal(optional.status(), 400, await optional.text());
      assert.match(await optional.text(), /grippers.*no gripper_driver/);
      assert.equal((await page.request.get(base + 'api/adapters/robots/openarmx_no_gripper')).status(), 400);
    } else {
      assert.equal(optional.status(), 201, await optional.text());
      assert.equal((await optional.json()).draft.gripper_driver, null);
    }
    assert.deepEqual([...new Set(mutations)], ['/api/adapters/robots', '/api/adapters/robots/openarmx_01/save',
      `/api/adapters/robots/openarmx_01/grippers/${encodeURIComponent(gripperName)}/test`]);
    assert.deepEqual(pageErrors, []);
    console.log('PASS: selected plugin payload, whitespace, field errors, copy, duplicate, optional gripper and reload persistence');
  } catch (error) {
    console.error('Browser errors:', pageErrors);
    console.error('Page notification:', await page.locator('#toast').textContent());
    console.error('Create state:', await page.evaluate(() => ({
      handler: String(document.querySelector('#createRobot').onclick),
      open: document.querySelector('#managedCreateDialog').open,
      display: getComputedStyle(document.querySelector('#managedCreateForm')).display,
      parent: document.querySelector('#managedCreateForm').parentElement.tagName,
    })));
    throw error;
  } finally {
    await browser.close();
  }
})().catch(error => {console.error(error); process.exitCode = 1;});
