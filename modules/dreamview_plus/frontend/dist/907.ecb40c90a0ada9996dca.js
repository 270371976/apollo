"use strict";(self.webpackChunk=self.webpackChunk||[]).push([[907],{70907:(e,t,n)=>{n.r(t),n.d(t,{default:()=>N});var r=n(40366),a=n.n(r),i=n(49161),o=n(15076),c=n(11446),l=n(23218);function u(){return(0,l.f2)((function(e){return{"panel-camera-root":{position:"relative",height:"100%",width:"100%",display:"flex",justifyContent:"center",alignItems:"center"},"camera-btn-container":{position:"absolute",bottom:"24px",right:"24px"},"camera-btn-item":{display:"inline-block",cursor:"pointer",textAlign:"center",width:"32px",height:"32px",lineHeight:"32px",background:"#343C4D",borderRadius:"6px",marginTop:"12px",fontSize:"16px"},"camera-canvas-container":{width:"100%",height:"100%",maxWidth:"100%",maxHeight:"100%",display:"flex",justifyContent:"center",alignItems:"center"},"panel-camera-canvas":{},"layer-menu-container":{width:"158px",height:"94px"},"layer-menu-header":{height:"40px",paddingLeft:"24px",display:"flex",alignItems:"center",borderBottom:"1px solid #383B45",fontFamily:"PingFangSC-Medium",fontSize:"16px",color:"#FFFFFF",fontWeight:"500"},"layer-menu-content-right":{height:"54px",paddingLeft:"24px",display:"flex",alignItems:"center",fontFamily:"PingFangSC-Regular",fontSize:"14px",color:"#A6B5CC",fontWeight:"400"}}}))()}var f=n(83517),s=new Map([["ST_UNKNOWN","rgba(96, 96, 96, 1.000)"],["ST_UNKNOWN_MOVABLE","rgba(96, 96, 96, 1.000)"],["ST_UNKNOWN_UNMOVABLE","rgba(96, 96, 96, 1.000)"],["ST_CAR","rgba(243, 187, 37, 1.000)"],["ST_VAN","rgba(243, 187, 37, 1.000)"],["ST_TRUCK","rgba(243, 187, 37, 1.000)"],["ST_BUS","rgba(243, 187, 37, 1.000)"],["ST_CYCLIST","rgba(231, 91, 135, 1.000)"],["ST_MOTORCYCLIST","rgba(231, 91, 135, 1.000)"],["ST_TRICYCLIST","rgba(231, 91, 135, 1.000)"],["ST_PEDESTRIAN","#35DACF"],["ST_TRAFFICCONE","#35DACF"]]),m=n(47960),y={"":{boundingbox:{defaultVisible:!1,currentVisible:!1,vizKey:"Boundingbox"}}};function b(e){return b="function"==typeof Symbol&&"symbol"==typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"==typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e},b(e)}function p(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function g(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?p(Object(n),!0).forEach((function(t){d(e,t,n[t])})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):p(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function d(e,t,n){var r;return r=function(e,t){if("object"!=b(e)||!e)return e;var n=e[Symbol.toPrimitive];if(void 0!==n){var r=n.call(e,"string");if("object"!=b(r))return r;throw new TypeError("@@toPrimitive must return a primitive value.")}return String(e)}(t),(t="symbol"==b(r)?r:r+"")in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}function h(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){var n=null==e?null:"undefined"!=typeof Symbol&&e[Symbol.iterator]||e["@@iterator"];if(null!=n){var r,a,i,o,c=[],l=!0,u=!1;try{if(i=(n=n.call(e)).next,0===t){if(Object(n)!==n)return;l=!1}else for(;!(l=(r=i.call(n)).done)&&(c.push(r.value),c.length!==t);l=!0);}catch(e){u=!0,a=e}finally{try{if(!l&&null!=n.return&&(o=n.return(),Object(o)!==o))return}finally{if(u)throw a}}return c}}(e,t)||function(e,t){if(e){if("string"==typeof e)return v(e,t);var n=Object.prototype.toString.call(e).slice(8,-1);return"Object"===n&&e.constructor&&(n=e.constructor.name),"Map"===n||"Set"===n?Array.from(e):"Arguments"===n||/^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)?v(e,t):void 0}}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method.")}()}function v(e,t){(null==t||t>e.length)&&(t=e.length);for(var n=0,r=new Array(t);n<t;n++)r[n]=e[n];return r}function S(e){var t=(0,f.d)().panelId,n=e.setShowBoundingBox,o=u(),l=o.classes,s=o.cx,b=(0,m.Bd)("layerMenu").t,p=h((0,r.useState)(y),2),v=p[0],S=p[1],w=h((0,r.useState)((function(){return Object.keys(v)})),1)[0],x=h((0,r.useState)(w[0]),2),E=x[0],O=x[1],j=h((0,r.useState)(v[E]),2),C=j[0],I=j[1],A=(0,c.Mj)("".concat(c.qK.layerMenu).concat(t)),T=(0,c.Mj)("".concat(c.qK.BBox).concat(t));return(0,r.useEffect)((function(){var e=A.get();e&&(S(e),O(w[0]),I(e[w[0]]))}),[]),a().createElement("div",{className:l["layer-menu-container"]},a().createElement("div",{className:l["layer-menu-header"]},a().createElement("div",{className:l["layer-menu-header-left"]},b("layerMenu")),a().createElement("div",{className:l["layer-menu-header-right"]})),a().createElement("div",{className:l["layer-menu-content"]},a().createElement("div",{className:l["layer-menu-content-left"]},w.map((function(e){return a().createElement("li",{key:e,className:s(l["layer-menu-content-left-li"],d({},l["li-active"],E===e)),onClick:function(){O(e),I(v[e])}},a().createElement("span",null,e))}))),a().createElement("div",{className:l["layer-menu-content-right"]},Object.keys(C).map((function(e){var t=v[E][e].currentVisible;return a().createElement("li",{key:e},a().createElement("span",null,a().createElement(i.Sc,{checked:t,defaultChecked:"1"===T.get(),onChange:function(t){var r=t.target.checked;n(r),T.set(r?1:0);var a=g(g({},v[E]),{},d({},e,g(g({},v[E][e]),{},{currentVisible:r}))),i=g(g({},v),{},d({},E,a));I((function(){return a})),S((function(){return i})),A.set(i)}},e)))})))))}const w=a().memo(S);var x=n(46533),E=n(60346),O=n(27878);function j(e){return j="function"==typeof Symbol&&"symbol"==typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"==typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e},j(e)}function C(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){var n=null==e?null:"undefined"!=typeof Symbol&&e[Symbol.iterator]||e["@@iterator"];if(null!=n){var r,a,i,o,c=[],l=!0,u=!1;try{if(i=(n=n.call(e)).next,0===t){if(Object(n)!==n)return;l=!1}else for(;!(l=(r=i.call(n)).done)&&(c.push(r.value),c.length!==t);l=!0);}catch(e){u=!0,a=e}finally{try{if(!l&&null!=n.return&&(o=n.return(),Object(o)!==o))return}finally{if(u)throw a}}return c}}(e,t)||function(e,t){if(e){if("string"==typeof e)return I(e,t);var n=Object.prototype.toString.call(e).slice(8,-1);return"Object"===n&&e.constructor&&(n=e.constructor.name),"Map"===n||"Set"===n?Array.from(e):"Arguments"===n||/^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)?I(e,t):void 0}}(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method.")}()}function I(e,t){(null==t||t>e.length)&&(t=e.length);for(var n=0,r=new Array(t);n<t;n++)r[n]=e[n];return r}function A(){var e=(0,f.d)(),t=e.logger,n=e.panelId,l=e.onPanelResize,m=e.initSubscription,y=(0,c.Mj)("".concat(c.qK.BBox).concat(n)),b=C((0,r.useState)((function(){return y.get()})),2),p=b[0],g=b[1],d=C((0,r.useState)(null),2),h=d[0],v=d[1],S=C((0,r.useState)(),2),E=S[0],I=S[1],A=(0,r.useRef)(null);(0,r.useEffect)((function(){var e,t,n,r;m((e={},t=x.lt.CAMERA,n={consumer:function(e){I(e)}},r=function(e,t){if("object"!=j(e)||!e)return e;var n=e[Symbol.toPrimitive];if(void 0!==n){var r=n.call(e,"string");if("object"!=j(r))return r;throw new TypeError("@@toPrimitive must return a primitive value.")}return String(e)}(t),(t="symbol"==j(r)?r:r+"")in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e))}),[]),(0,r.useLayoutEffect)((function(){l((function(e,t){v({viewPortWidth:e,viewPortHeight:t})}))}),[]),(0,r.useEffect)((function(){var e;if(!(0,o.isEmpty)(E)&&(null==E||null===(e=E.image)||void 0===e?void 0:e.length)>0){var r=E.kImageScale,a=new Uint8Array(E.image),i=new Blob([a],{type:"image/jpeg"});createImageBitmap(i).then((function(e){var a=A.current;if(a){var i={},o=e.width/e.height>h.viewPortWidth/h.viewPortHeight,c=h.viewPortWidth/e.width,l=h.viewPortHeight/e.height,u=o?c:l;i.drawImageWidth=e.width*u,i.drawImageHeight=e.height*u,a.width=i.drawImageWidth,a.height=i.drawImageHeight;var f=a.getContext("2d");f.drawImage(e,0,0,i.drawImageWidth,i.drawImageHeight),E.bbox2d&&E.bbox2d.length>0&&p&&(t.debug(n,"has obstacles"),E.bbox2d.forEach((function(e,a){var i=E.obstaclesId[a],o=E.obstaclesSubType[a];f.strokeStyle=s.get(o)||"red";var c=e.xmin,l=e.ymin,m=e.xmax,y=e.ymax;if(c!==l||l!==m||m!==y){var b=C([c,l,m,y].map((function(e){return e*(null!=r?r:1)*u})),4);c=b[0],l=b[1],m=b[2],y=b[3],f.strokeRect(c,l,m-c,y-l),f.fillStyle=s.get(o)||"white",f.font="12px Arial",f.fillText("".concat(o.substring(3),":").concat(i),c,l)}else t.debug(n,"bbox is empty")})))}})).catch((function(e){t.error(e)}))}}),[E,p,h]);var T=u().classes;return a().createElement("div",{className:T["panel-camera-root"]},a().createElement("div",{className:T["camera-btn-container"]},a().createElement(i.AM,{placement:"leftTop",content:a().createElement(w,{setShowBoundingBox:g}),trigger:"click"},a().createElement("span",{className:T["camera-btn-item"]},a().createElement(i.Yx,null)))),a().createElement(O.A,{className:T["camera-canvas-container"]},a().createElement("canvas",{ref:A,id:"camera-".concat(n),className:T["panel-camera-canvas"]})))}function T(e){var t=(0,r.useMemo)((function(){return(0,E.A)({PanelComponent:A,panelId:e.panelId,subscribeInfo:[{name:x.lt.CAMERA,needChannel:!0}]})}),[]);return a().createElement(t,e)}A.displayName="InternalCameraView";const N=a().memo(T)}}]);