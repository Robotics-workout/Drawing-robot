#include "web_server.h"

// External references
extern WebServer server;
extern QueueHandle_t stepperQueue;
extern QueueHandle_t penQueue;

// Process vector paths and send commands to queues
void processVectorPaths(JsonArray paths) {
  float lastX = 0, lastY = 0;
  
  // First pass: find bounding box of all points
  float minX = 1e6, maxX = -1e6, minY = 1e6, maxY = -1e6;
  bool hasPoints = false;
  
  for (JsonArray path : paths) {
    for (JsonVariant point : path) {
      if (point.is<JsonArray>() && point.size() >= 2) {
        float x = point[0].as<float>();
        float y = point[1].as<float>();
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
        hasPoints = true;
      }
    }
  }
  
  // If no valid points, return
  if (!hasPoints) {
    Serial.println("No valid points found in paths");
    return;
  }
  
  // Calculate scale to fit
  float srcWidth = maxX - minX;
  float srcHeight = maxY - minY;
  float margin = 10.0;
  float targetWidth = (drawing_area_x_limits[1] - drawing_area_x_limits[0]) - (2 * margin);
  float targetHeight = (drawing_area_y_limits[1] - drawing_area_y_limits[0]) - (2 * margin);
  
  float scale = 1.0;
  if (srcWidth > 0 && srcHeight > 0) {
    float scaleX = targetWidth / srcWidth;
    float scaleY = targetHeight / srcHeight;
    scale = (scaleX < scaleY) ? scaleX : scaleY;
  }
  
  // Second pass: scale and send points
  for (JsonArray path : paths) {
    bool pathStarted = false;
    
    for (JsonVariant point : path) {
      if (point.is<JsonArray>() && point.size() >= 2) {
        float x = point[0].as<float>();
        float y = point[1].as<float>();
        
        // Scale to fit drawing area
        if (srcWidth > 0 && srcHeight > 0) {
          float scaledX = (x - minX) * scale + margin + drawing_area_x_limits[0];
          float scaledY = (y - minY) * scale + margin + drawing_area_y_limits[0];
          x = scaledX;
          y = scaledY;
        } else {
          // Single point or degenerate case - center it
          x = (drawing_area_x_limits[0] + drawing_area_x_limits[1]) / 2;
          y = (drawing_area_y_limits[0] + drawing_area_y_limits[1]) / 2;
        }
        
        // Ensure coordinates are within bounds (safety check)
        x = constrain(x, drawing_area_x_limits[0], drawing_area_x_limits[1]);
        y = constrain(y, drawing_area_y_limits[0], drawing_area_y_limits[1]);
        
        if (!pathStarted) {
          // First point: lift pen and move to position
          PenCommand penUpCmd;
          penUpCmd.state = PEN_UP;
          penUpCmd.isValid = true;
          xQueueSend(penQueue, &penUpCmd, portMAX_DELAY);
          delay(100); // Wait for pen to lift
          
          StepperMoveCommand moveCmd;
          moveCmd.x = x;
          moveCmd.y = y;
          moveCmd.isValid = true;
          xQueueSend(stepperQueue, &moveCmd, portMAX_DELAY);
          
          pathStarted = true;
        } else {
          // Drawing point: lower pen and move
          PenCommand penDownCmd;
          penDownCmd.state = PEN_DOWN;
          penDownCmd.isValid = true;
          xQueueSend(penQueue, &penDownCmd, portMAX_DELAY);
          delay(100); // Wait for pen to lower
          
          StepperMoveCommand moveCmd;
          moveCmd.x = x;
          moveCmd.y = y;
          moveCmd.isValid = true;
          xQueueSend(stepperQueue, &moveCmd, portMAX_DELAY);
        }
        
        lastX = x;
        lastY = y;
      }
    }
    
    // Lift pen after each path
    if (pathStarted) {
      PenCommand penUpCmd;
      penUpCmd.state = PEN_UP;
      penUpCmd.isValid = true;
      xQueueSend(penQueue, &penUpCmd, portMAX_DELAY);
    }
  }
}

// Handle POST request to /draw
void handleDraw() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    Serial.println("Received drawing data");
    
    DynamicJsonDocument doc(8192);
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
      return;
    }
    
    // Clear queues
    xQueueReset(stepperQueue);
    xQueueReset(penQueue);
    
    // Process vector paths and get scaled paths for visualization
    if (doc.containsKey("paths") && doc["paths"].is<JsonArray>()) {
      // Build response with scaled paths for simulation
      DynamicJsonDocument responseDoc(16384);
      responseDoc["status"] = "ok";
      responseDoc["message"] = "Drawing started";
      JsonArray responsePaths = responseDoc.createNestedArray("paths");
      
      JsonArray inputPaths = doc["paths"].as<JsonArray>();
      
      // First pass: find bounding box
      float minX = 1e6, maxX = -1e6, minY = 1e6, maxY = -1e6;
      bool hasPoints = false;
      
      for (JsonArray path : inputPaths) {
        for (JsonVariant point : path) {
          if (point.is<JsonArray>() && point.size() >= 2) {
            float x = point[0].as<float>();
            float y = point[1].as<float>();
            if (x < minX) minX = x;
            if (x > maxX) maxX = x;
            if (y < minY) minY = y;
            if (y > maxY) maxY = y;
            hasPoints = true;
          }
        }
      }
      
      if (!hasPoints) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No valid points found\"}");
        return;
      }
      
      // Calculate scale
      float srcWidth = maxX - minX;
      float srcHeight = maxY - minY;
      float margin = 10.0;
      float targetWidth = (drawing_area_x_limits[1] - drawing_area_x_limits[0]) - (2 * margin);
      float targetHeight = (drawing_area_y_limits[1] - drawing_area_y_limits[0]) - (2 * margin);
      
      float scale = 1.0;
      if (srcWidth > 0 && srcHeight > 0) {
        float scaleX = targetWidth / srcWidth;
        float scaleY = targetHeight / srcHeight;
        scale = (scaleX < scaleY) ? scaleX : scaleY;
      }
      
      // Process and scale paths for both robot and visualization
      for (JsonArray inputPath : inputPaths) {
        JsonArray scaledPath = responsePaths.createNestedArray();
        bool pathStarted = false;
        
        for (JsonVariant point : inputPath) {
          if (point.is<JsonArray>() && point.size() >= 2) {
            float x = point[0].as<float>();
            float y = point[1].as<float>();
            
            // Scale to fit drawing area
            float scaledX, scaledY;
            if (srcWidth > 0 && srcHeight > 0) {
              scaledX = (x - minX) * scale + margin + drawing_area_x_limits[0];
              scaledY = (y - minY) * scale + margin + drawing_area_y_limits[0];
            } else {
              scaledX = (drawing_area_x_limits[0] + drawing_area_x_limits[1]) / 2;
              scaledY = (drawing_area_y_limits[0] + drawing_area_y_limits[1]) / 2;
            }
            
            // Ensure within bounds
            scaledX = constrain(scaledX, drawing_area_x_limits[0], drawing_area_x_limits[1]);
            scaledY = constrain(scaledY, drawing_area_y_limits[0], drawing_area_y_limits[1]);
            
            // Add to response for visualization
            JsonArray pathPoint = scaledPath.createNestedArray();
            pathPoint.add(scaledX);
            pathPoint.add(scaledY);
            
            // Send to robot queues
            if (!pathStarted) {
              PenCommand penUpCmd;
              penUpCmd.state = PEN_UP;
              penUpCmd.isValid = true;
              xQueueSend(penQueue, &penUpCmd, portMAX_DELAY);
              delay(100);
              
              StepperMoveCommand moveCmd;
              moveCmd.x = scaledX;
              moveCmd.y = scaledY;
              moveCmd.isValid = true;
              xQueueSend(stepperQueue, &moveCmd, portMAX_DELAY);
              pathStarted = true;
            } else {
              PenCommand penDownCmd;
              penDownCmd.state = PEN_DOWN;
              penDownCmd.isValid = true;
              xQueueSend(penQueue, &penDownCmd, portMAX_DELAY);
              delay(100);
              
              StepperMoveCommand moveCmd;
              moveCmd.x = scaledX;
              moveCmd.y = scaledY;
              moveCmd.isValid = true;
              xQueueSend(stepperQueue, &moveCmd, portMAX_DELAY);
            }
          }
        }
        
        // Lift pen after each path
        if (pathStarted) {
          PenCommand penUpCmd;
          penUpCmd.state = PEN_UP;
          penUpCmd.isValid = true;
          xQueueSend(penQueue, &penUpCmd, portMAX_DELAY);
        }
      }
      
      String response;
      serializeJson(responseDoc, response);
      server.send(200, "application/json", response);
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No paths found\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

// Handle GET request to /
void handleRoot() {
  // Robot parameters for JavaScript
  float base = BASE;
  float drawingXMin = drawing_area_x_limits[0];
  float drawingXMax = drawing_area_x_limits[1];
  float drawingYMin = drawing_area_y_limits[0];
  float drawingYMax = drawing_area_y_limits[1];
  float drawingHeight = DRAWING_AREA_HEIGHT;
  
  String html = "<!DOCTYPE html><html><head><title>Drawing Robot</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:Arial,sans-serif;margin:0;padding:10px;background:#f5f5f5;}";
  html += ".main-container{display:flex;gap:20px;max-width:1400px;margin:0 auto;}";
  html += ".left-panel{flex:1;min-width:350px;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}";
  html += ".right-panel{flex:1;min-width:500px;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}";
  html += "h1{margin-top:0;color:#333;}";
  html += "button{background:#4CAF50;color:white;padding:10px 20px;border:none;cursor:pointer;margin:5px;border-radius:4px;font-size:14px;}";
  html += "button:hover{background:#45a049;}";
  html += "button:disabled{background:#ccc;cursor:not-allowed;}";
  html += "textarea{width:100%;height:250px;margin:10px 0;padding:8px;border:1px solid #ddd;border-radius:4px;font-family:monospace;font-size:12px;box-sizing:border-box;}";
  html += "a{color:#4CAF50;text-decoration:none;font-weight:bold;}";
  html += "a:hover{text-decoration:underline;}";
  html += ".info{background:#f0f0f0;padding:10px;border-radius:5px;margin:10px 0;font-size:13px;}";
  html += ".info p{margin:5px 0;}";
  html += "#canvas{border:2px solid #ddd;border-radius:4px;background:#fafafa;display:block;width:100%;}";
  html += ".status{background:#e3f2fd;padding:8px;border-radius:4px;margin:10px 0;font-size:13px;}";
  html += ".status.error{background:#ffebee;color:#c62828;}";
  html += ".status.success{background:#e8f5e9;color:#2e7d32;}";
  html += "@media (max-width:900px){.main-container{flex-direction:column;}}</style></head><body>";
  
  html += "<div class='main-container'>";
  
  // Left panel - Controls
  html += "<div class='left-panel'>";
  html += "<h1>Drawing Robot Controller</h1>";
  
  // Connection info
  html += "<div class='info'>";
  if (WiFi.getMode() == WIFI_AP) {
    html += "<p><strong>Mode:</strong> Access Point</p>";
    html += "<p><strong>IP:</strong> " + WiFi.softAPIP().toString() + "</p>";
  } else {
    html += "<p><strong>Mode:</strong> Connected to WiFi</p>";
    html += "<p><strong>IP:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>Network:</strong> " + String(WiFi.SSID()) + "</p>";
  }
  html += "</div>";
  
  html += "<p><strong>Send vector paths in JSON format:</strong></p>";
  html += "<form id='drawForm'>";
  html += "<textarea id='paths' name='paths' placeholder='{\"paths\":[[[100,100],[200,100],[200,200],[100,200],[100,100]]]}'></textarea>";
  html += "<button type='submit' id='submitBtn'>Start Drawing</button>";
  html += "<button type='button' id='clearBtn'>Clear Simulation</button>";
  html += "</form>";
  html += "<div id='status'></div>";
  html += "<hr style='margin:20px 0;'>";
  html += "<p><strong>OTA Update:</strong> <a href='/update' target='_blank'>Click here to upload firmware</a></p>";
  html += "</div>"; // end left-panel
  
  // Right panel - Simulation
  html += "<div class='right-panel'>";
  html += "<h1>Robot Simulation</h1>";
  html += "<canvas id='canvas' width='600' height='800'></canvas>";
  html += "<div style='margin-top:10px;font-size:12px;color:#666;'>";
  html += "Drawing Area: " + String(drawingXMin, 0) + "-" + String(drawingXMax, 0) + "mm × " + String(drawingYMin, 0) + "-" + String(drawingYMax, 0) + "mm";
  html += "</div>";
  html += "</div>"; // end right-panel
  
  html += "</div>"; // end main-container
  
  // JavaScript
  html += "<script>";
  html += "const BASE=" + String(base, 1) + ";";
  html += "const DRAW_X_MIN=" + String(drawingXMin, 1) + ";";
  html += "const DRAW_X_MAX=" + String(drawingXMax, 1) + ";";
  html += "const DRAW_Y_MIN=" + String(drawingYMin, 1) + ";";
  html += "const DRAW_Y_MAX=" + String(drawingYMax, 1) + ";";
  html += "const DRAW_HEIGHT=" + String(drawingHeight, 1) + ";";
  html += "const canvas=document.getElementById('canvas');";
  html += "const ctx=canvas.getContext('2d');";
  html += "let currentPaths=[];";
  html += "let animationPaths=[];";
  html += "let currentX=0,currentY=0;";
  html += "let penDown=false;";
  html += "let animating=false;";
  html += "let animationFrame=0;";
  html += "let animationInterval=null;";
  html += "const ANIMATION_SPEED=1000;";
  html += "function setupCanvas(){";
  html += "const scale=Math.min(canvas.width/(BASE+100),canvas.height/(DRAW_HEIGHT+100));";
  html += "ctx.setTransform(1,0,0,1,0,0);";
  html += "ctx.clearRect(0,0,canvas.width,canvas.height);";
  html += "const offsetX=50;const offsetY=50;";
  html += "ctx.save();";
  html += "ctx.translate(offsetX,offsetY);";
  html += "ctx.scale(scale,scale);";
  html += "ctx.strokeStyle='#ccc';ctx.fillStyle='#e0e0e0';ctx.lineWidth=2;";
  html += "ctx.beginPath();ctx.rect(0,0,BASE,DRAW_HEIGHT);ctx.fill();ctx.stroke();";
  html += "ctx.strokeStyle='#2196F3';ctx.setLineDash([5,5]);ctx.lineWidth=1.5;";
  html += "ctx.beginPath();ctx.rect(DRAW_X_MIN,DRAW_Y_MIN,DRAW_X_MAX-DRAW_X_MIN,DRAW_Y_MAX-DRAW_Y_MIN);";
  html += "ctx.stroke();ctx.setLineDash([]);";
  html += "ctx.fillStyle='#333';ctx.strokeStyle='#000';ctx.lineWidth=3;";
  html += "ctx.beginPath();ctx.arc(0,0,20,0,Math.PI*2);ctx.fill();ctx.stroke();";
  html += "ctx.beginPath();ctx.arc(BASE,0,20,0,Math.PI*2);ctx.fill();ctx.stroke();";
  html += "ctx.fillStyle='#666';ctx.font='12px Arial';ctx.textAlign='center';";
  html += "ctx.fillText('L',0,35);ctx.fillText('R',BASE,35);";
  html += "ctx.restore();";
  html += "}";
  html += "function drawFrame(){";
  html += "if(!animating||animationPaths.length===0)return;";
  html += "setupCanvas();";
  html += "const scale=Math.min(canvas.width/(BASE+100),canvas.height/(DRAW_HEIGHT+100));";
  html += "const offsetX=50;const offsetY=50;";
  html += "ctx.save();";
  html += "ctx.translate(offsetX,offsetY);";
  html += "ctx.scale(scale,scale);";
  html += "let pointCount=0;";
  html += "let currentPathIdx=0;";
  html += "let currentPointInPath=0;";
  html += "animationPaths.forEach((path,pathIdx)=>{";
  html += "if(path.length===0)return;";
  html += "const pathStartPoint=pointCount;";
  html += "const pathEndPoint=pointCount+path.length;";
  html += "if(animationFrame>=pathEndPoint){";
  html += "ctx.strokeStyle=pathIdx%2===0?'#f44336':'#2196F3';";
  html += "ctx.lineWidth=2;";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(path[0][0],path[0][1]);";
  html += "for(let i=1;i<path.length;i++){ctx.lineTo(path[i][0],path[i][1]);}";
  html += "ctx.stroke();";
  html += "if(path.length>0){currentX=path[path.length-1][0];currentY=path[path.length-1][1];penDown=true;}";
  html += "}else if(animationFrame>pathStartPoint){";
  html += "const pointsToDraw=animationFrame-pathStartPoint;";
  html += "ctx.strokeStyle=pathIdx%2===0?'#f44336':'#2196F3';";
  html += "ctx.lineWidth=2;";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(path[0][0],path[0][1]);";
  html += "for(let i=1;i<pointsToDraw&&i<path.length;i++){ctx.lineTo(path[i][0],path[i][1]);}";
  html += "ctx.stroke();";
  html += "if(pointsToDraw>0&&pointsToDraw<=path.length){";
  html += "currentX=path[pointsToDraw-1][0];currentY=path[pointsToDraw-1][1];penDown=true;}";
  html += "}";
  html += "pointCount+=path.length;";
  html += "});";
  html += "if(currentX>0||currentY>0){";
  html += "ctx.fillStyle=penDown?'#f44336':'#4CAF50';";
  html += "ctx.strokeStyle='#000';ctx.lineWidth=1.5;";
  html += "ctx.beginPath();ctx.arc(currentX,currentY,6,0,Math.PI*2);ctx.fill();ctx.stroke();";
  html += "}";
  html += "ctx.restore();";
  html += "animationFrame++;";
  html += "let totalPoints=0;";
  html += "animationPaths.forEach(p=>{totalPoints+=p.length;});";
  html += "if(animationFrame>=totalPoints){";
  html += "clearInterval(animationInterval);";
  html += "animating=false;";
  html += "drawAllPaths();";
  html += "}";
  html += "}";
  html += "function drawAllPaths(){";
  html += "const scale=Math.min(canvas.width/(BASE+100),canvas.height/(DRAW_HEIGHT+100));";
  html += "const offsetX=50;const offsetY=50;";
  html += "setupCanvas();";
  html += "ctx.save();";
  html += "ctx.translate(offsetX,offsetY);";
  html += "ctx.scale(scale,scale);";
  html += "currentPaths.forEach((path,pathIdx)=>{";
  html += "if(path.length===0)return;";
  html += "ctx.strokeStyle=pathIdx%2===0?'#f44336':'#2196F3';";
  html += "ctx.lineWidth=2;";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(path[0][0],path[0][1]);";
  html += "for(let i=1;i<path.length;i++){ctx.lineTo(path[i][0],path[i][1]);}";
  html += "ctx.stroke();";
  html += "});";
  html += "ctx.restore();";
  html += "}";
  html += "function startAnimation(paths){";
  html += "if(animating){clearInterval(animationInterval);}";
  html += "animationPaths=paths;";
  html += "animationFrame=0;";
  html += "currentX=0;currentY=0;penDown=false;";
  html += "animating=true;";
  html += "animationInterval=setInterval(drawFrame,ANIMATION_SPEED);";
  html += "}";
  html += "function showStatus(msg,isError=false){";
  html += "const status=document.getElementById('status');";
  html += "status.className='status '+(isError?'error':'success');";
  html += "status.textContent=msg;";
  html += "}";
  html += "document.getElementById('drawForm').onsubmit=async function(e){";
  html += "e.preventDefault();";
  html += "const btn=document.getElementById('submitBtn');";
  html += "btn.disabled=true;";
  html += "showStatus('Sending drawing data...');";
  html += "try{";
  html += "const input=document.getElementById('paths').value;";
  html += "const data={paths:JSON.parse(input).paths};";
  html += "const res=await fetch('/draw',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});";
  html += "const result=await res.json();";
  html += "if(result.status==='ok'&&result.paths){";
  html += "currentPaths=result.paths;";
  html += "showStatus('Drawing started successfully! Animation starting...');";
  html += "startAnimation(result.paths);";
  html += "}else{";
  html += "showStatus('Error: '+(result.message||'Unknown error'),true);";
  html += "}";
  html += "}catch(err){";
  html += "showStatus('Error: '+err.message,true);";
  html += "}";
  html += "btn.disabled=false;";
  html += "};";
  html += "document.getElementById('clearBtn').onclick=function(){";
  html += "if(animating){clearInterval(animationInterval);animating=false;}";
  html += "currentPaths=[];animationPaths=[];currentX=0;currentY=0;penDown=false;animationFrame=0;";
  html += "setupCanvas();";
  html += "showStatus('Simulation cleared');";
  html += "};";
  html += "setupCanvas();";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

// Handle GET request to /status
void handleStatus() {
  String status = "{\"status\":\"ready\",\"stepper_queue_size\":";
  status += uxQueueMessagesWaiting(stepperQueue);
  status += ",\"pen_queue_size\":";
  status += uxQueueMessagesWaiting(penQueue);
  status += "}";
  server.send(200, "application/json", status);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/draw", HTTP_POST, handleDraw);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();
  Serial.println("HTTP server started");
}


