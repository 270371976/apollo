"use strict";(self.webpackChunk=self.webpackChunk||[]).push([[323],{80323:(e,t,n)=>{n.r(t),n.d(t,{default:()=>P});var r=n(40366),o=n.n(r),a=n(49161),i=n(88219),l=n(47960),c=n(23218);function u(e){return u="function"==typeof Symbol&&"symbol"==typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"==typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e},u(e)}function s(e,t){var n=Object.keys(e);if(Object.getOwnPropertySymbols){var r=Object.getOwnPropertySymbols(e);t&&(r=r.filter((function(t){return Object.getOwnPropertyDescriptor(e,t).enumerable}))),n.push.apply(n,r)}return n}function f(e){for(var t=1;t<arguments.length;t++){var n=null!=arguments[t]?arguments[t]:{};t%2?s(Object(n),!0).forEach((function(t){var r,o,a,i;r=e,o=t,a=n[t],i=function(e,t){if("object"!=u(e)||!e)return e;var n=e[Symbol.toPrimitive];if(void 0!==n){var r=n.call(e,"string");if("object"!=u(r))return r;throw new TypeError("@@toPrimitive must return a primitive value.")}return String(e)}(o),(o="symbol"==u(i)?i:i+"")in r?Object.defineProperty(r,o,{value:a,enumerable:!0,configurable:!0,writable:!0}):r[o]=a})):Object.getOwnPropertyDescriptors?Object.defineProperties(e,Object.getOwnPropertyDescriptors(n)):s(Object(n)).forEach((function(t){Object.defineProperty(e,t,Object.getOwnPropertyDescriptor(n,t))}))}return e}function m(){return(0,c.f2)((function(e){return{"panel-console-root":{padding:"4px ".concat(e.tokens.padding.speace2),height:"100%",width:"100%"},"panel-console-inner":{minWidth:"244px"},"panel-console-monitor":{display:"flex",marginBottom:e.tokens.margin.speace,"&:last-of-type":{marginBottom:"0"}},"panel-console-monitor-icon":{display:"flex",marginTop:"-1px",fontSize:e.components.panelConsole.iconFontSize},"panel-console-monitor-text":f(f({marginLeft:e.tokens.margin.speace,marginRight:e.tokens.margin.speace2},e.tokens.typography.content),{},{flex:1}),"panel-console-monitor-time":{whiteSpace:"nowrap"},"panel-console-monitor-item":{},error:{color:e.tokens.colors.error2},info:{color:e.tokens.colors.brand3},warn:{color:e.tokens.colors.warn2}}}))()}var p=n(83517),y=n(46533),b=n(60346),v=n(27878);function d(e){return d="function"==typeof Symbol&&"symbol"==typeof Symbol.iterator?function(e){return typeof e}:function(e){return e&&"function"==typeof Symbol&&e.constructor===Symbol&&e!==Symbol.prototype?"symbol":typeof e},d(e)}function S(e,t){return function(e){if(Array.isArray(e))return e}(e)||function(e,t){var n=null==e?null:"undefined"!=typeof Symbol&&e[Symbol.iterator]||e["@@iterator"];if(null!=n){var r,o,a,i,l=[],c=!0,u=!1;try{if(a=(n=n.call(e)).next,0===t){if(Object(n)!==n)return;c=!1}else for(;!(c=(r=a.call(n)).done)&&(l.push(r.value),l.length!==t);c=!0);}catch(e){u=!0,o=e}finally{try{if(!c&&null!=n.return&&(i=n.return(),Object(i)!==i))return}finally{if(u)throw o}}return l}}(e,t)||g(e,t)||function(){throw new TypeError("Invalid attempt to destructure non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method.")}()}function g(e,t){if(e){if("string"==typeof e)return h(e,t);var n=Object.prototype.toString.call(e).slice(8,-1);return"Object"===n&&e.constructor&&(n=e.constructor.name),"Map"===n||"Set"===n?Array.from(e):"Arguments"===n||/^(?:Ui|I)nt(?:8|16|32)(?:Clamped)?Array$/.test(n)?h(e,t):void 0}}function h(e,t){(null==t||t>e.length)&&(t=e.length);for(var n=0,r=new Array(t);n<t;n++)r[n]=e[n];return r}function E(e,t,n){var r;return r=function(e,t){if("object"!=d(e)||!e)return e;var n=e[Symbol.toPrimitive];if(void 0!==n){var r=n.call(e,"string");if("object"!=d(r))return r;throw new TypeError("@@toPrimitive must return a primitive value.")}return String(e)}(t),(t="symbol"==d(r)?r:r+"")in e?Object.defineProperty(e,t,{value:n,enumerable:!0,configurable:!0,writable:!0}):e[t]=n,e}var O=function(e){return e.ERROR="ERROR",e.FATAL="FATAL",e.FAIL="FAIL",e.SUCCESS="SUCCESS",e.UNKNOWN="UNKNOWN",e}(O||{}),w=E(E(E(E(E({},O.ERROR,"error"),O.FATAL,"error"),O.FAIL,"error"),O.SUCCESS,"info"),O.UNKNOWN,"warn"),j=E(E(E(E(E({},O.ERROR,o().createElement(a.hG,null)),O.FATAL,o().createElement(a.hG,null)),O.FAIL,o().createElement(a.hG,null)),O.SUCCESS,o().createElement(a.zW,null)),O.UNKNOWN,o().createElement(a.zW,null));function A(e){var t=m(),n=t.classes,r=t.cx,i=e.level,l=e.text,c=e.time,u=j[i]||o().createElement(a.zW,null),s=w[i]||"warn";return o().createElement("li",{className:r(n["panel-console-monitor"],n[s])},o().createElement("span",{className:n["panel-console-monitor-icon"]},u),o().createElement("span",{className:n["panel-console-monitor-text"]},l),o().createElement("span",{className:n["panel-console-monitor-time"]},c))}var N=o().memo(A);function C(){var e=(0,p.d)(),t=(e.data,e.initSubscription),n=(e.setKeyDownHandlers,S((0,r.useState)([]),2)),a=n[0],c=n[1],u=S((0,r.useState)(),2),s=u[0],f=u[1];(0,l.Bd)("panels").t,(0,r.useEffect)((function(){t(E({},y.lt.SIM_WORLD,{consumer:function(e){f(e)}}))}),[]),(0,r.useEffect)((function(){var e;null==s||null===(e=s.notification)||void 0===e||e.forEach((function(e){!function(e){c((function(t){var n,r=[e].concat(function(e){if(Array.isArray(e))return h(e)}(n=t)||function(e){if("undefined"!=typeof Symbol&&null!=e[Symbol.iterator]||null!=e["@@iterator"])return Array.from(e)}(n)||g(n)||function(){throw new TypeError("Invalid attempt to spread non-iterable instance.\nIn order to be iterable, non-array objects must have a [Symbol.iterator]() method.")}());return r.length>29&&r.pop(),r}))}({level:e.item.logLevel,text:e.item.msg,time:(0,i.eh)(1e3*e.timestampSec)})}))}),[s]);var b=m().classes;return o().createElement(v.A,{className:b["panel-console-root"]},o().createElement("div",{className:b["panel-console-inner"]},a.map((function(e,t){return o().createElement(N,{key:t+1,text:e.text,level:e.level,time:e.time})}))))}function I(e){var t=(0,r.useMemo)((function(){return(0,b.A)({PanelComponent:C,panelId:e.panelId,subscribeInfo:[{name:y.lt.SIM_WORLD,needChannel:!1}]})}),[]);return o().createElement(t,e)}C.displayName="InternalConsole";const P=o().memo(I)},88219:(e,t,n)=>{function r(e){var t=arguments.length>1&&void 0!==arguments[1]?arguments[1]:2,n=!(arguments.length>2&&void 0!==arguments[2])||arguments[2],r=Number(e);if(r>Math.pow(10,t-1))return String(r);var o="0".repeat(t-String(r).length);if("number"!=typeof r)throw new Error("fill0 recived an invidate value");return n?"".concat(o).concat(r):"".concat(r).concat(o)}function o(e){var t=arguments.length>1&&void 0!==arguments[1]&&arguments[1],n=new Date(e),o=r(n.getHours()),a=r(n.getMinutes()),i=r(n.getSeconds()),l=r(n.getMilliseconds(),3),c="".concat(o,":").concat(a,":").concat(i);return t&&(c+=":".concat(l)),c}n.d(t,{Dy:()=>l,_E:()=>r,eh:()=>o});var a=1e3,i=6e4;function l(e){var t=r(Math.floor(e%1e3),3),n=r(Math.floor(e/a%60)),o=r(Math.floor(e/i));return"".concat(o,":").concat(n,".").concat(t)}}}]);