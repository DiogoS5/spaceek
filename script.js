// Create markers for rooms
function createMarkers(roomsData) {
    // Clear existing markers
    markers.forEach(marker => map.removeLayer(marker));
    markers = [];
    
    roomsData.forEach(room => {
        if (room.location && typeof room.location.latitude === 'number' && typeof room.location.longitude === 'number') {
            const ocupacao = (room.curOcc && room.maxOcc) ? Math.round((room.curOcc / room.maxOcc) * 100) : 0;
            const occupancyCategory = getOccupancyCategory(ocupacao);
            const isFavorite = userFavorites.includes(room.id);
            const isSelected = room.id === selectedRoomId;
            
            // Skip if filtering by favorites and not a favorite
            if (currentFilter === 'favorites' && !isFavorite) return;
            
            // Skip if filtering by occupancy category
            if (['low', 'medium', 'high'].includes(currentFilter) && currentFilter !== occupancyCategory) return;
            
            // Create marker with much more visible highlighting for selected room
            const markerIcon = L.divIcon({
                className: `custom-marker ${isSelected ? 'highlighted-marker' : ''}`,
                html: `<div style="
                    background-color: ${getOccupancyColor(ocupacao)}; 
                    width: 100%; 
                    height: 100%; 
                    border-radius: 50%; 
                    border: ${isSelected ? '4px solid #3b82f6' : '2px solid white'}; 
                    ${isSelected ? 'box-shadow: 0 0 15px #3b82f6, 0 0 30px rgba(59, 130, 246, 0.5);' : ''}
                    ${isFavorite ? 'box-shadow: 0 0 0 2px #f59e0b;' : ''}
                "></div>`,
                iconSize: isSelected ? [40, 40] : [20, 20],
                iconAnchor: isSelected ? [20, 20] : [10, 10]
            });
            
            const marker = L.marker([room.location.latitude, room.location.longitude], {
                icon: markerIcon,
                title: room.name || 'Sala',
                zIndexOffset: isSelected ? 1000 : 0
            }).addTo(map);
            
            // Store reference to selected room marker
            if (isSelected) {
                selectedRoomMarker = marker;
            }
            
            // Resto do código para o popup e eventos...
            
            markers.push(marker);
        }
    });
}

// Exemplo de highlightMarker alternativo
function highlightMarker(roomId) {
    // Remove destaque de todos os marcadores
    window.markers.forEach(m => {
        if (m._icon) {
            m._icon.style.border = "";
            m._icon.style.background = "";
        }
    });

    // Realça só o marcador correspondente
    const marker = window.markers.find(m => m.options && m.options.roomId == roomId);
    if (marker && marker._icon) {
        marker._icon.style.border = "4px solid #a78bfa";
        marker._icon.style.background = "#ede9fe";
        setTimeout(() => {
            marker._icon.style.border = "";
            marker._icon.style.background = "";
        }, 2000);
    }
}

// Only keep this ONCE in your file!
const realisticAffluence = [
  // 0-5: early morning (very low)
  2, 2, 2, 3, 4, 6,
  // 6-8: morning (increasing)
  12, 22, 38,
  // 9-11: late morning (peak)
  60, 75, 85,
  // 12-13: lunch (decrease)
  50, 35,
  // 14-17: afternoon (increase again)
  55, 65, 75, 80,
  // 18-19: stabilize (high)
  80, 75,
  // 20-23: evening/night (decrease)
  40, 20, 10, 5
];

// Returns an array of 24 values, updating only if 24h have passed for each hour
function getTICAffluence24h() {
  const key = 'tic_affluence_histogram';
  const now = Date.now();
  let data = JSON.parse(localStorage.getItem(key));
  if (!data || !Array.isArray(data.values) || !Array.isArray(data.timestamps)) {
    data = {
      values: realisticAffluence.slice(),
      timestamps: Array(24).fill(now)
    };
    localStorage.setItem(key, JSON.stringify(data));
  } else {
    for (let hour = 0; hour < 24; hour++) {
      if (now - data.timestamps[hour] > 24 * 60 * 60 * 1000) {
        // Keep in 0-100 range
        let newVal = realisticAffluence[hour] + Math.floor(Math.random() * 11 - 5); // +/-5%
        newVal = Math.max(0, Math.min(100, newVal));
        data.values[hour] = newVal;
        data.timestamps[hour] = now;
      }
    }
    localStorage.setItem(key, JSON.stringify(data));
  }
  return data.values;
}

// Usage in your histogram rendering logic:
function getAffluenceForRoom(roomName) {
  if (roomName === "TIC") {
    return getTICAffluence24h();
  }
  // ...your logic for other rooms...
}

// Example: in your renderHourlyChart or splitview logic
// let averages = getAffluenceForRoom(roomData.name);