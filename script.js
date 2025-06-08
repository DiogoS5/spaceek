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

document.querySelectorAll('.detalhes-btn').forEach(btn => {
  btn.addEventListener('click', async function() {
    const container = this.closest('.room-card').querySelector('.details-container');
    const isOpening = container.style.display === 'none';

    // Toggle visibility
    container.style.display = isOpening ? 'block' : 'none';

    // Only load data when first opening
    if (isOpening && !container.dataset.loaded) {
      const roomId = this.closest('.room-card').dataset.roomId;

      // Fetch hourly data from Firestore
      const hourlyData = await fetchHourlyData(roomId);

      // Render the chart
      const chartContainer = container.querySelector('.hourly-chart');
      if (chartContainer) {
        renderHourlyChart(chartContainer, hourlyData);
      }
      container.dataset.loaded = 'true';
    }
  });
});

// Fetch hourly data from Firestore
async function fetchHourlyData(roomId) {
  const docRef = doc(db, "salas", roomId);
  const docSnap = await getDoc(docRef);
  return docSnap.exists() ? (docSnap.data().hourlyAverages || Array(24).fill(0)) : Array(24).fill(0);
}

// Render the hourly histogram
function renderHourlyChart(container, averages) {
  const now = new Date();
  const currentHour = now.getHours();

  container.innerHTML = averages.map((avg, hour) => `
    <div class="chart-bar">
      <div class="bar${hour === currentHour ? ' current-hour' : ''}" 
           style="height: ${(avg || 0)}%; background: ${getOcupacaoColor(avg)};">
      </div>
      <span class="bar-label">${hour}h</span>
    </div>
  `).join('');
}

// Color helper
function getOcupacaoColor(percent) {
  if (percent >= 75) return 'var(--color-danger)';
  if (percent >= 50) return 'var(--color-warning)';
  return 'var(--color-success)';
}