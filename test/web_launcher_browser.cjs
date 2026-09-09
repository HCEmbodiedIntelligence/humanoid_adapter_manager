/* Only the temporary server with substituted harmless child processes is used. */
const assert=require('node:assert/strict');
let chromium;
try{({chromium}=require(process.env.HUMANOID_TEST_PLAYWRIGHT_MODULE||'playwright'));}
catch(error){if(process.env.HUMANOID_TEST_PLAYWRIGHT_MODULE)throw error;process.exit(77);}
const base=process.argv[2];
assert.equal(new URL(base).hostname,'127.0.0.1');
(async()=>{
  let browser;
  try{browser=await chromium.launch({headless:true,executablePath:process.env.HUMANOID_TEST_BROWSER_EXECUTABLE||undefined});}
  catch(error){if(!process.env.HUMANOID_TEST_BROWSER_EXECUTABLE&&error.message.includes("Executable doesn't exist")){process.exitCode=77;return;}throw error;}
  const page=await browser.newPage({viewport:{width:1440,height:1000}}),errors=[],mutations=[];
  try{
    page.setDefaultTimeout(15000);
    page.on('pageerror',error=>errors.push(error.message));
    page.on('dialog',dialog=>dialog.accept());
    page.on('request',request=>{if(request.method()==='POST')mutations.push(new URL(request.url()).pathname);});
    await page.goto(base+'dashboard/#robots');
    await page.waitForFunction(()=>!document.querySelector('#startManagedRobot').disabled);
    assert.deepEqual(mutations,[],'Opening the page must not start hardware');
    assert.equal(await page.locator('#applyRobot').isVisible(),false);
    await page.locator('#startManagedRobot').click();
    await page.waitForFunction(()=>!document.querySelector('#stopManagedRobot').disabled);
    let state=await (await page.request.get(base+'api/launcher')).json();
    const original=state.runtime.revision;
    assert.equal(state.runtime.phase,'running');
    await page.locator('#managedRobotName').fill('网页重启测试');
    assert.equal(await page.locator('#restartManagedRobot').isDisabled(),true);
    const saved=page.waitForResponse(r=>r.request().method()==='POST'&&r.url().endsWith('/save'));
    await page.locator('#saveRobotConfig').click();
    assert.equal((await saved).status(),200);
    await page.waitForFunction(()=>!document.querySelector('#restartManagedRobot').disabled);
    await page.waitForFunction(()=>document.querySelector('#toast').textContent.includes('重启机器人后生效'));
    state=await (await page.request.get(base+'api/launcher')).json();
    assert.equal(state.runtime.revision,original,'Saving must not change running revision');
    assert.equal(mutations.filter(path=>path.startsWith('/api/launcher/')).length,1);
    await page.locator('#restartManagedRobot').click();
    await page.waitForFunction(()=>document.querySelector('#launcherMessage').textContent.includes('重启命令已完成'));
    state=await (await page.request.get(base+'api/launcher')).json();
    assert.notEqual(state.runtime.revision,original);
    await page.locator('#stopManagedRobot').click();
    await page.waitForFunction(()=>!document.querySelector('#startManagedRobot').disabled);
    assert.equal((await page.request.get(base+'dashboard/')).status(),200);
    assert.match(await page.locator('#launcherMessage').textContent(),/网页保持运行/);
    if(process.env.HUMANOID_TEST_SCREENSHOT_DIR){
      await page.screenshot({path:process.env.HUMANOID_TEST_SCREENSHOT_DIR+'/launcher-desktop.png',fullPage:false});
      await page.setViewportSize({width:390,height:844});
      await page.screenshot({path:process.env.HUMANOID_TEST_SCREENSHOT_DIR+'/launcher-mobile.png',fullPage:false});
      assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,'No horizontal overflow');
    }
    assert.deepEqual(errors,[]);
    console.log('PASS: start/save/restart/stop without web reload; saved revision activated only on restart');
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
