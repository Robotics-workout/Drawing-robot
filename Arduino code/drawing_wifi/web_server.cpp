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
    
    // Process vector paths
    if (doc.containsKey("paths") && doc["paths"].is<JsonArray>()) {
      processVectorPaths(doc["paths"].as<JsonArray>());
      server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Drawing started\"}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No paths found\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No data received\"}");
  }
}

// Handle GET request to /
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Drawing Robot</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:20px;}";
  html += "button{background:#4CAF50;color:white;padding:10px 20px;border:none;cursor:pointer;margin:5px;}";
  html += "button:hover{background:#45a049;}";
  html += ".container{max-width:600px;margin:0 auto;}";
  html += "textarea{width:100%;height:200px;margin:10px 0;}";
  html += "a{color:#4CAF50;text-decoration:none;font-weight:bold;}";
  html += "a:hover{text-decoration:underline;}";
  html += ".info{background:#f0f0f0;padding:10px;border-radius:5px;margin:10px 0;}</style></head><body>";
  html += "<div class='container'><h1>Drawing Robot Controller</h1>";
  
  // Display connection info
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
  
  html += "<p>Send vector paths in JSON format:</p>";
  html += "<form id='drawForm'><textarea id='paths' name='paths' placeholder='{\"paths\":[[[100,100],[200,100],[200,200],[100,200],[100,100]]]}'></textarea>";
  html += "<br><button type='submit'>Start Drawing</button></form>";
  html += "<hr><p><strong>OTA Update:</strong> <a href='/update'>Click here to upload firmware</a></p>";
  html += "<script>document.getElementById('drawForm').onsubmit=async function(e){";
  html += "e.preventDefault();const data={paths:JSON.parse(document.getElementById('paths').value).paths};";
  html += "const res=await fetch('/draw',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});";
  html += "alert(await res.text());};</script></body></html>";
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

