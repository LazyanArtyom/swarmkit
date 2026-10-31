/* VECR dissertation motion atlas. Conceptual SVG scenes, no remote resources or fabricated measurements.
   The separate site.js replay renders frozen experiment traces. */
(() => {
  'use strict';
  const NS = 'http://www.w3.org/2000/svg';
  const G = '#65dec7', A = '#f5b777', B = '#85b7ff', W = '#e9f6f6', M = '#8baab8';
  const clamp = n => Math.max(0, Math.min(1, n));
  const lerp = (a,b,t) => a+(b-a)*clamp(t);
  const smooth = t => {t=clamp(t);return t*t*(3-2*t);};
  const f = n => Number(n).toFixed(1);
  const find = (root,id) => root.querySelector(`[data-el="${id}"]`);
  const setText = (root,id,v) => { const e=find(root,id);if(e)e.textContent=v; };
  const attr = (root,id,name,v) => { const e=find(root,id);if(e)e.setAttribute(name,String(v)); };
  const pos = (root,id,x,y,rotate=0) => attr(root,id,'transform',`translate(${f(x)} ${f(y)}) rotate(${f(rotate)})`);
  const route = (y=205) => `<rect x="80" y="${y-35}" width="780" height="70" rx="30" fill="#19423f" opacity=".65"/><path d="M95 ${y}H845" stroke="#305f59" stroke-width="3" stroke-dasharray="9 11"/><circle cx="95" cy="${y}" r="9" fill="${B}"/><circle cx="845" cy="${y}" r="9" fill="${G}"/><text x="84" y="${y+66}" fill="${M}" font-size="15">A · 0 m</text><text x="800" y="${y+66}" fill="${M}" font-size="15">B · 40 m</text>`;
  const drone = (id,color=G) => `<g data-el="${id}" transform="translate(95 205)"><circle r="30" fill="${color}" opacity=".10"/><g stroke="${color}" stroke-width="3" fill="#09232d"><path d="M-19-19L19 19M19-19L-19 19"/><circle cx="-23" cy="-23" r="9"/><circle cx="23" cy="-23" r="9"/><circle cx="-23" cy="23" r="9"/><circle cx="23" cy="23" r="9"/><path d="M0-16L13 6L0 2L-13 6Z" fill="${color}" stroke="none"/></g><circle r="5" fill="#ecfaf8"/></g>`;
  const label = (x,y,value,color=M,size=14) => `<text x="${x}" y="${y}" fill="${color}" font-size="${size}" font-family="system-ui, sans-serif">${value}</text>`;
  const scenes = {
    hero: {
      title:'One automatic recovery: detect, return and resume BEFORE B',
      note:'Conceptual timeline. The UAV never flies to B before this deviation correction; the exact target depends on accepted evidence and safety bounds.',
      stages:['Follow A → B','Detect deviation','Return and re-enter','Resume toward B'],breakpoints:[0,.23,.46,.70,1],seconds:14,
      svg:()=>`${label(34,39,'REAL-TIME LOCAL CORRECTION · A → B',W,17)}${label(34,66,'The deviation is qualified while B is still ahead.',M,14)}${route(196)}
        <rect x="350" y="175" width="91" height="42" rx="6" fill="${A}" opacity=".20" stroke="${A}" stroke-width="2" stroke-dasharray="6 5"/>
        <rect data-el="heroCoveredA" x="95" y="182" width="0" height="28" rx="3" fill="${G}" opacity=".85"/>
        <rect data-el="heroReacquired" x="350" y="182" width="0" height="28" fill="${G}" opacity=".87"/>
        <rect data-el="heroCoveredLater" x="441" y="182" width="0" height="28" fill="${G}" opacity=".7"/>
        <path d="M330 196 Q375 99 449 112" stroke="${A}" stroke-width="3" fill="none" stroke-dasharray="6 5"/>
        <path d="M449 112 Q420 127 350 196" stroke="${B}" stroke-width="3" fill="none" stroke-dasharray="6 5"/>
        <circle data-el="heroSample1" cx="369" cy="154" r="7" fill="${A}" opacity="0"/>
        <circle data-el="heroSample2" cx="405" cy="119" r="7" fill="${A}" opacity="0"/>
        <circle data-el="heroSample3" cx="449" cy="112" r="7" fill="${A}" opacity="0"/>
        ${label(330,155,'q = earlier of p and gap',A,14)}<text data-el="heroWarning" x="534" y="113" fill="${A}" font-size="15">CORRECTION BEGINS BEFORE B</text>
        ${drone('heroDrone',G)}<rect x="30" y="292" width="880" height="43" rx="10" fill="#123846" stroke="#3a6270"/>
        <text data-el="heroStatus" x="49" y="319" fill="${W}" font-size="16">Monitor accepted coverage and corridor compliance</text>`,
      draw:(svg,p)=>{
        let x,y,angle=0;
        if(p<.23){x=lerp(95,330,p/.23);y=196;}
        else if(p<.36){const t=smooth((p-.23)/.13);x=lerp(330,449,t);y=lerp(196,112,t);angle=-32;}
        else if(p<.46){x=449;y=112;angle=-32;}
        else if(p<.64){const t=smooth((p-.46)/.18);x=lerp(449,350,t);y=lerp(112,196,t);angle=135;}
        else if(p<.70){x=350;y=196;}
        else {x=lerp(350,845,smooth((p-.70)/.30));y=196;}
        pos(svg,'heroDrone',x,y,angle);
        for(const [i,t] of [[1,.25],[2,.31],[3,.36]])attr(svg,'heroSample'+i,'opacity',p>t?'0.9':'0');
        attr(svg,'heroCoveredA','width',255*smooth(p/.24));
        attr(svg,'heroCoveredLater','width',p>.70?404*smooth((p-.70)/.30):0);
        attr(svg,'heroReacquired','width',p>.70?91*smooth((p-.70)/.10):0);
        attr(svg,'heroWarning','opacity',p>.3&&p<.7?'1':'0');
        setText(svg,'heroStatus',p<.23?'MOVING · survey evidence accumulates':p<.36?'Qualified violations accumulate with hysteresis':p<.46?'DEVIATING → RECOVERING · B has NOT been reached':p<.64?'Return to q = min(current progress, first required gap)':p<.70?'Check bounded stopped re-entry, not just command ACK':p<.97?'Resume MOVING toward B, retaining accepted intervals':'At B: report arrival + any residual gaps; NO rewind');
      },
      summary:p=>p<.23?'The higher-level graph algorithm assigns A → B; the local supervisor collects evidence.':p<.46?'The UAV departs its corridor and VECR starts correction before arriving at B.':p<.7?'The vehicle returns to a conservative evidence-selected coordinate, verifies physical re-entry, then may resume.':'At destination, any unresolved intervals are handed to the caller—not automatically revisited.'
    },
    graph: {
      title:'Graph algorithm and physical executor have different jobs', note:'Conceptual SDK boundary; rotor-router is a motivating example, not an implemented planner.',
      stages:['Assign A → B','Execute locally','Arrive + report','Caller decides'],breakpoints:[0,.15,.63,.85,1],seconds:12,
      svg:() =>`${label(42,42,'EXTERNAL GRAPH CONTROLLER',B,14)}${label(516,42,'VECR LOCAL EXECUTOR',G,14)}<rect x="32" y="62" width="418" height="229" rx="14" fill="#112e42" stroke="#35546b"/><rect x="484" y="62" width="426" height="229" rx="14" fill="#10372f" stroke="#315d55"/>
      <g stroke="#7194ad" stroke-width="3" fill="none"><path d="M120 156L252 111L358 168L234 244L120 156M252 111L234 244"/></g><g fill="#183e59" stroke="${B}" stroke-width="2"><circle cx="120" cy="156" r="21"/><circle cx="252" cy="111" r="21"/><circle cx="358" cy="168" r="21"/><circle cx="234" cy="244" r="21"/></g>
      ${label(113,162,'A',W,17)}${label(245,117,'B',W,17)}${label(351,175,'C',W,17)}${label(227,251,'D',W,17)}<path data-el="graphEdge" d="M120 156L252 111" stroke="${A}" stroke-width="7"/><g data-el="graphChip" transform="translate(120 156)"><circle r="10" fill="${A}" stroke="#fdf4e5" stroke-width="2"/></g>
      <rect x="524" y="130" width="340" height="36" rx="9" fill="#2c5a51"/><rect data-el="graphCoverage" x="524" y="130" width="0" height="36" rx="9" fill="${G}" opacity=".80"/><text x="531" y="116" fill="${W}" font-size="14">Assigned physical edge A → B</text><text data-el="graphOutcome" x="516" y="235" fill="${A}" font-size="14">Caller observes physical execution</text><text x="516" y="267" fill="${M}" font-size="12">VECR never chooses the next graph edge</text>`,
      draw:(svg,p)=>{const n=p<.63?smooth(p/.63):1;pos(svg,'graphChip',lerp(120,252,n),lerp(156,111,n));attr(svg,'graphCoverage','width',320*smooth((p-.15)/.65));attr(svg,'graphEdge','stroke',p>.84?G:A);setText(svg,'graphOutcome',p<.63?'A → B in execution; poll status on demand':p<.85?'ARRIVED + residual interval report':'Caller alone decides: continue / reassign / retry');},
      summary:p=>p<.3?'The planner chooses the next graph edge; the SDK does not implement rotor-router.':p<.63?'VECR runs local motion supervision and qualified evidence tracking.':p<.85?'At B, VECR reports both arrival and coverage state, even when gaps remain.':'Any further task is a new mission-level choice, not an implicit VECR backward command.'
    },
    contract: {
      title:'At B: report unresolved intervals without returning',note:'Conceptual example of the implemented terminal arrival callback; no autonomous endpoint return.',
      stages:['Command accepted','Travel to B','Find residual gap','Emit callback'],seconds:12,
      svg:()=>`${label(40,42,'ARRIVAL IS NOT THE SAME AS COVERAGE',W,17)}${['COMMAND ACK','ARRIVED AT B','COVERAGE STATE','TERMINAL REPORT'].map((t,i)=>`<rect data-el="contractBox${i}" x="${34+i*226}" y="68" width="205" height="62" rx="10" fill="#183343" stroke="#4b6570" stroke-width="1.5"/>${label(45+i*226,105,t,M,13)}`).join('')}${route(228)}
      <rect x="95" y="216" width="310" height="24" rx="4" fill="${G}" opacity=".8"/><rect x="525" y="216" width="320" height="24" rx="4" fill="${G}" opacity=".8"/><rect x="405" y="213" width="120" height="30" rx="4" stroke="${A}" stroke-dasharray="6 4" fill="${A}" opacity=".30"/>
      ${label(382,197,'unverified',A,14)}${drone('contractDrone')}<text data-el="contractOutcome" x="120" y="320" fill="${G}" font-size="16">Only the caller may authorize a new edge task.</text>`,
      draw:(svg,p)=>{const phase=[p>.1,p>.38,p>.63,p>.85];phase.forEach((on,i)=>{attr(svg,'contractBox'+i,'fill',on?'#1c6457':'#183343');attr(svg,'contractBox'+i,'stroke',on?G:'#4b6570');});pos(svg,'contractDrone',lerp(95,845,smooth(p/.54)),228);setText(svg,'contractOutcome',p<.6?'B is a destination, not an instruction to backtrack':p<.85?'Coverage = INCOMPLETE · [18.2, 21.6] m':'CALLBACK: ARRIVED / INCOMPLETE · NO GOTO after B');},
      summary:p=>p<.38?'Command acceptance does not prove physical arrival or image quality.':p<.63?'The UAV reaches B and stays at B; the middle interval is unverified.':p<.85?'VECR assembles the immutable arrival/coverage result.':'One terminal event is delivered; the caller decides how to resolve the gap. VECR sends no return command.'
    },
    footprint: {
      title:'Why uncertainty shrinks an accepted camera footprint',note:'Geometric demonstration only · fixed calibrated field-of-view values and truthful quality assertions are assumed; no image pixels are inspected.',
      stages:['Measured footprint','Apply error bounds','Contract interval','Qualify capture'], seconds:13,
      options:[['tight','Small error bounds'],['wide','Larger error bounds']],
      svg:()=>`${label(40,43,'CAMERA ACQUISITION VS. SAFE CREDIT',W,17)}${label(40,74,'Blue: possible geometric footprint      Green: conservatively accepted interval',M,14)}
        <rect x="72" y="148" width="796" height="103" rx="12" fill="#163f41" stroke="#43746b"/><path d="M72 200H868" stroke="#487c74" stroke-dasharray="9 8"/>
        <rect x="259" y="148" width="418" height="103" rx="8" fill="${B}" opacity=".19" stroke="${B}" stroke-width="2"/>
        <rect data-el="safeFootprint" x="459" y="151" width="0" height="97" rx="5" fill="${G}" opacity=".72"/>
        <g data-el="cameraDrone" transform="translate(466 104)"><path d="M-23 0H23M0-23V23" stroke="${B}" stroke-width="6"/><circle r="15" fill="#14384a" stroke="${B}" stroke-width="3"/><circle r="5" fill="${G}"/></g>
        <path d="M466 114L259 148M466 114L677 148" fill="none" stroke="${B}" stroke-width="2" stroke-dasharray="5 6"/>
        ${label(71,281,'0 m',M,13)}${label(805,281,'along-edge interval',M,13)}<text data-el="footprintNote" x="82" y="318" fill="${G}" font-size="17">Verify bounded geometry before accepting coverage</text>`,
      draw:(svg,p,mode)=>{const width=(mode==='wide'?170:330)*smooth((p-.2)/.55);attr(svg,'safeFootprint','width',width);attr(svg,'safeFootprint','x',468-width/2);setText(svg,'footprintNote',p<.7?'Contraction accounts for declared localization / altitude error bounds':mode==='wide'?'Larger bounds → narrower conservative interval':'Smaller bounds → wider qualified interval (within the physical footprint)');},
      summary:(p,mode)=>p<.25?'A physical camera footprint is not automatically credited to the mission.':p<.70?'The agent contracts a fixed calibrated footprint using hard position and altitude error bounds.':mode==='wide'?'More uncertainty means less accepted coverage; an invalid cross-track fit rejects the capture.':'The bounded, contracted interval contributes to the accepted union only after qualification.'
    },
    states: {
      title:'A recovery command does not mean recovery happened',note:'Illustrative state transition · runtime phases include MOVING, DEVIATING, RECOVERING, ARRIVED, FAILED, CANCELLED and UNKNOWN',
      stages:['MOVING','DEVIATING','RECOVERING','Outcome'], breakpoints:[0,.17,.38,.93,1],seconds:12,
      options:[['success','Qualified recovery'],['rejected','Command rejected']],
      svg:()=>`${label(44,40,'PHYSICAL STATE, NOT COMMAND STATUS',W,17)}<path d="M120 130H824" stroke="#345767" stroke-width="4" stroke-dasharray="5 10"/>${['MOVING','DEVIATING','RECOVERING','MOVING','ARRIVED'].map((s,i)=>`<g><circle data-el="state${i}" cx="${120+i*176}" cy="130" r="38" fill="#173644" stroke="#476371" stroke-width="2"/><text data-el="stateLabel${i}" x="${120+i*176}" y="136" fill="${W}" text-anchor="middle" font-size="${s==='RECOVERING'?11:12}" font-family="system-ui">${s}</text></g>`).join('')}<rect x="92" y="225" width="750" height="65" rx="12" fill="#123342" stroke="#3e6471"/><rect data-el="stateProgress" x="92" y="225" width="0" height="65" rx="12" fill="${G}" opacity=".26"/><text data-el="stateCondition" x="112" y="262" fill="${W}" font-size="17">Qualified violations with hysteresis trigger DEVIATING.</text>`,
      draw:(svg,p,mode)=>{const reject=mode==='rejected';const active=reject&&p>.54?4:p<.17?0:p<.38?1:p<.78?2:p<.93?3:4;setText(svg,'stateLabel4',reject?'FAILED':'ARRIVED');for(let i=0;i<5;i++){const on=i===active;attr(svg,'state'+i,'fill',on?(reject&&i===4?'#7c4930':'#195d54'):'#173644');attr(svg,'state'+i,'stroke',on?(reject&&i===4?A:G):'#476371');}attr(svg,'stateProgress','width',p*750);setText(svg,'stateCondition',reject&&p>.54?'Command rejected → FAILED; no arrival.':p<.17?'MOVING · accumulating verified intervals.':p<.38?'DEVIATING · sustained departure detected.':p<.55?'RECOVERING · command accepted, physical return still pending.':p<.78?'RECOVERING · re-entry dwell and monitoring required.':p<.93?'MOVING · resumed after physical re-entry.':'ARRIVED · stopped arrival qualified; survey verdict separate.');},
      summary:(p,mode)=>mode==='rejected'&&p>.54?'A rejected return command terminates the attempt; ACK cannot substitute for physical dwell.':p<.38?'Physical conformance samples—not a UI action—trigger recovery.':p<.78?'RECOVERING persists until stationary re-entry is genuinely qualified.':'The outcome is determined by verified observations and safety conditions.'
    },
    evidence: {
      title:'What the agent believes versus what was acquired',note:'Schematic truth-boundary demonstration · dishonest metadata is a deliberate adversarial control',
      stages:['Capture report','Bounds check','Compare truth','Decision'], seconds:12,
      options:[['honest','Honest bounded sensor'],['dishonest','Dishonest metadata']],
      svg:()=>`${label(41,42,'AGENT-ACCEPTED EVIDENCE',G,15)}${label(41,192,'INDEPENDENT PHYSICAL TRUTH',B,15)}${[100,248].map(y=>`<rect x="76" y="${y-22}" width="788" height="55" rx="8" fill="#143342" stroke="#507084"/><path d="M76 ${y+5}H864" stroke="#557b80" stroke-dasharray="8 7"/>`).join('')}<rect data-el="evidenceAcceptedA" x="76" y="78" width="0" height="55" rx="6" fill="${G}" opacity=".7"/><rect data-el="evidenceAcceptedB" x="410" y="78" width="0" height="55" rx="6" fill="${G}" opacity=".7"/><rect data-el="evidenceTruthA" x="76" y="226" width="0" height="55" rx="6" fill="${B}" opacity=".6"/><rect data-el="evidenceTruthB" x="410" y="226" width="0" height="55" rx="6" fill="${B}" opacity=".6"/><rect data-el="evidenceTruthGap" x="326" y="226" width="84" height="55" fill="${A}" opacity="0" stroke="${A}" stroke-dasharray="6 5"/><text data-el="evidenceVerdict" x="445" y="319" text-anchor="middle" font-size="16" fill="${W}">Identity and error bounds checked locally</text>`,
      draw:(svg,p,mode)=>{const dishonest=mode==='dishonest',v=smooth(p/.74);attr(svg,'evidenceAcceptedA','width',334*v);attr(svg,'evidenceAcceptedB','width',454*v);attr(svg,'evidenceTruthA','width',250*v);attr(svg,'evidenceTruthB','width',454*v);attr(svg,'evidenceTruthA','width',(dishonest?250:334)*v);attr(svg,'evidenceTruthGap','opacity',dishonest?.35:0);setText(svg,'evidenceVerdict',p<.74?'Check capture metadata, geometry, bounds, and attempt identity':dishonest?'FALSE FULFILLMENT POSSIBLE · metadata is not physical truth':'Conditional containment: accepted footprint ⊆ actual footprint');attr(svg,'evidenceVerdict','fill',dishonest&&p>.74?A:G);},
      summary:(p,mode)=>mode==='dishonest'?'With dishonest acquisition metadata, a reported full strip may conceal a real hole. Authentication alone cannot establish physical truth.':p<.74?'The agent contracts accepted footprints using declared deterministic error bounds.':'Under truthful metadata and valid hard bounds, accepted coverage is contained within actual coverage.'
    },
    wait: {
      title:'Two reporting interfaces: snapshot on demand, one terminal event',note:'Conceptual API requirement. This is a pull/status and push/completion demonstration, not a measurement of callback traffic.',
      stages:['Execution active','Caller polls','Arrival or failure','Terminal callback'],breakpoints:[0,.2,.44,.74,1],seconds:12,
      options:[['incomplete','Arrive with a gap'],['failure','Correction fails']],
      svg:()=>`${label(38,40,'EXECUTION STATUS SNAPSHOT · PULL',B,16)}<rect x="42" y="60" width="856" height="100" rx="12" fill="#123744" stroke="#466c7c"/>
       <text data-el="waitPoll" x="64" y="96" fill="${W}" font-size="16">GetEdgeStatus(drone, attempt): MOVING</text><text data-el="waitInterval" x="64" y="130" fill="${M}" font-size="15">Unverified [18.2,21.6]m; attempts 1/2</text>
       ${label(38,201,'TERMINAL RESULT · PUSH ONCE',G,16)}<rect x="42" y="214" width="856" height="93" rx="12" fill="#123b36" stroke="#45766e"/><text data-el="waitCallback" x="64" y="260" fill="${G}" font-size="16">No callback during each telemetry update</text><text data-el="waitCallbackSub" x="64" y="291" fill="${M}" font-size="14">Only the final edge outcome notifies the caller</text>`,
      draw:(svg,p,mode)=>{const failure=mode==='failure';setText(svg,'waitPoll',p<.44?'GetEdgeStatus(drone, attempt) → MOVING':p<.74?(failure?'GetEdgeStatus(drone, attempt) → RECOVERING':'GetEdgeStatus(drone, attempt) → MOVING; B dwell pending'):(failure?'GetEdgeStatus(drone, attempt) → FAILED':'GetEdgeStatus(drone, attempt) → ARRIVED'));
       setText(svg,'waitInterval',p<.74?'Verified [0,18.2], [21.6,28]m; gap still tracked':'Unverified [18.2,21.6]m · immutable evidence cutoff');
       setText(svg,'waitCallback',p<.74?'No callback on each telemetry frame':failure?'ONE EVENT: FAILED · reason: re-entry rejected':'ONE EVENT: ARRIVED · survey incomplete');
       setText(svg,'waitCallbackSub',p<.74?'The caller can poll when it wants':failure?'Budget or safety failure ends local control':'Caller may retry / reassign / continue · VECR sends no GOTO');},
      summary:(p,mode)=>p<.44?'The upper-level algorithm pulls a status snapshot only when desired.':p<.74?'The SDK maintains phase, missing intervals and finite recovery counts internally; it does not flood the planner with step events.':mode==='failure'?'A failed correction creates one terminal FAILED event with reason and partial evidence.':'Arrival at B generates one terminal ARRIVED event with a separate survey verdict, without a return-to-gap command.'
    }
  };
  let counter=0;
  const all=[];
  const reduced=window.matchMedia('(prefers-reduced-motion: reduce)');
  function build(kind,{compact=false}={}) {
    const s=scenes[kind],id='vecr-motion-'+(++counter),card=document.createElement('div');
    card.className='motion-atlas'+(compact?' motion-atlas--hero':'');card.dataset.scene=kind;
    const options=s.options?`<div class="motion-modes" role="group" aria-label="${s.title} scenario">${s.options.map(([key,title],i)=>`<button type="button" data-mode="${key}" aria-pressed="${i===0}">${title}</button>`).join('')}</div>`:'';
    card.innerHTML=`<div class="motion-head"><div><span class="motion-eyebrow">${compact?'THE IDEA IN MOTION':'INTERACTIVE RESEARCH EXPLAINER · CONCEPTUAL'}</span><h4>${s.title}</h4></div><span class="motion-live" aria-hidden="true">CONCEPTUAL ANIMATION</span></div>
     <div class="motion-screen"><svg viewBox="0 0 940 350" role="img" aria-label="Animated illustrative diagram: ${s.title}"><defs><pattern id="${id}-grid" width="28" height="28" patternUnits="userSpaceOnUse"><path d="M28 0H0V28" fill="none" stroke="#31505e" stroke-width=".6" opacity=".45"/></pattern></defs><rect width="940" height="350" rx="14" fill="#0b2331"/><rect width="940" height="350" rx="14" fill="url(#${id}-grid)"/>${s.svg()}</svg></div>
     <div class="motion-narrative"><span class="motion-step">01 / 04</span><p class="motion-summary" aria-live="off"></p></div>${options}
     <div class="motion-controls"><button type="button" data-action="play" class="motion-play" aria-label="Play animation">▶ <span>Play</span></button><button type="button" data-action="restart" aria-label="Restart animation" class="motion-restart">↺</button><label class="motion-slider-label"><span class="sr-only">Animation progress</span><input type="range" min="0" max="1000" value="0" step="1" aria-label="Scrub animation timeline"></label><span class="motion-time">00:00</span><button type="button" data-action="step" class="motion-next" title="Next stage">Next step →</button></div>
     <div class="motion-footnote">${s.note}</div>`;
    const svg=card.querySelector('svg'),slider=card.querySelector('input[type="range"]'),play=card.querySelector('[data-action="play"]'),restart=card.querySelector('[data-action="restart"]'),next=card.querySelector('[data-action="step"]'),sum=card.querySelector('.motion-summary'),step=card.querySelector('.motion-step'),time=card.querySelector('.motion-time');
    const modeButtons=[...card.querySelectorAll('[data-mode]')];let mode=modeButtons[0]?.dataset.mode||'default',p=0,playing=false,visible=false,raf=0,last=0,userPaused=false;
    const milestones=s.breakpoints||[0,.25,.5,.75,1];
    function paint(){s.draw(svg,p,mode);sum.textContent=s.summary(p,mode);let stage=3;for(let i=0;i<4;i++){if(p<milestones[i+1]){stage=i;break;}}step.textContent=`${String(stage+1).padStart(2,'0')} / 04 · ${s.stages[stage]}`;slider.value=String(Math.round(p*1000));time.textContent=`${String(Math.floor(p*s.seconds/60)).padStart(2,'0')}:${String(Math.floor(p*s.seconds%60)).padStart(2,'0')}`;slider.setAttribute('aria-valuetext',`${Math.round(p*100)} percent, ${s.stages[stage]}`);}
    function showPlaying(flag){playing=flag;play.querySelector('span').textContent=flag?'Pause':'Play';play.setAttribute('aria-label',flag?'Pause animation':'Play animation');play.firstChild.textContent=flag?'Ⅱ ':'▶ ';card.classList.toggle('is-playing',flag);}
    function pause(user=false){if(user)userPaused=true;showPlaying(false);cancelAnimationFrame(raf);raf=0;}
    function tick(ts){if(!playing)return;if(last){p+=Math.min(80,ts-last)/(s.seconds*1000);if(p>=1){p=1;pause();}}last=ts;paint();if(playing)raf=requestAnimationFrame(tick);}
    function start(){if(p>=1)p=0;userPaused=false;last=0;showPlaying(true);raf=requestAnimationFrame(tick);}
    play.addEventListener('click',()=>{if(playing)pause(true);else start();});
    restart.addEventListener('click',()=>{pause(true);p=0;paint();});
    next.addEventListener('click',()=>{pause(true);const milestone=milestones.find(q=>q>p+.001);p=milestone??0;paint();});
    slider.addEventListener('input',()=>{pause(true);p=Number(slider.value)/1000;paint();});
    modeButtons.forEach(b=>b.addEventListener('click',()=>{mode=b.dataset.mode;modeButtons.forEach(x=>x.setAttribute('aria-pressed',String(x===b)));pause(true);p=0;paint();}));
    const instance={card,pause,autostart:()=>{if(!playing&&!userPaused&&p<1&&!reduced.matches)start();},get visible(){return visible;},set visible(v){visible=v;v?this.autostart():pause(false);}};
    all.push(instance);paint();return card;
  }
  // Technical SVG references remain secondary to the conceptual interactive scenes.
  function replaceFigure(file,kind) {
    const button=document.querySelector(`.figure-open[data-image="assets/diagrams/${file}.svg"]`);
    if(!button)return;
    const figure=button.closest('figure');if(!figure)return;
    const shell=document.createElement('div');shell.className='motion-section';shell.append(build(kind));
    const details=document.createElement('details');details.className='motion-reference';
    const summary=document.createElement('summary');summary.textContent='Open the static technical reference diagram';
    details.append(summary);figure.replaceWith(shell);details.append(figure);shell.append(details);
  }
  replaceFigure('contract','contract');replaceFigure('states','states');replaceFigure('evidence','evidence');
  // Architecture is a technical reference diagram: keep it static.
  function afterHeading(id,kind){const h=document.getElementById(id);if(!h)return;const host=document.createElement('div');host.className='motion-section';host.append(build(kind));const next=h.nextElementSibling; // Insert after first explanatory paragraph when possible.
    if(next&&next.tagName==='P')next.insertAdjacentElement('afterend',host);else h.insertAdjacentElement('afterend',host);
  }
  afterHeading('foundations','graph');
  afterHeading('conservative-sensing-intervals','footprint');
  afterHeading('reporting-interfaces','wait');
  afterHeading('case-a-deviation-while-traveling-a-b','hero');
  // Hero illustration becomes a real-time concept animation. The research chart stays unchanged.
  const hero=document.querySelector('.hero-motion-anchor');
  if(hero){const panel=build('hero',{compact:true});hero.replaceWith(panel);}
  // Readers advance the concepts themselves; recorded replay has its own controls.
  document.addEventListener('visibilitychange',()=>{if(document.hidden)all.forEach(i=>i.pause(false));});
  reduced.addEventListener?.('change',()=>{if(reduced.matches)all.forEach(i=>i.pause(false));});
})();
