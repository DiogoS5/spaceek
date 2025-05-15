// Função para alternar o menu em dispositivos móveis
function toggleMenu() {
  const menu = document.querySelector('.menu');
  menu.classList.toggle('active');
}

// Função para criar um mapa de exemplo (para a página de mapa)
function initMap() {
  // Verifica se estamos na página do mapa
  const mapContainer = document.getElementById('map-container');
  if (!mapContainer) return;

  // Obtém os parâmetros da URL
  const urlParams = new URLSearchParams(window.location.search);
  const salaName = urlParams.get('sala');
  const localName = urlParams.get('local');

  // Se temos parâmetros, mostra informações sobre a sala
  if (salaName && localName) {
      const infoElement = document.createElement('div');
      infoElement.className = 'map-info';
      infoElement.innerHTML = `
          <h3>Localização: ${salaName}</h3>
          <p>${localName}</p>
          <p>Coordenadas simuladas: 41.1579, -8.6291</p>
      `;
      mapContainer.appendChild(infoElement);
  }

  // Aqui você pode adicionar código para inicializar um mapa real
  // como Google Maps, Leaflet, etc.
}

// Executa quando o DOM estiver carregado
document.addEventListener('DOMContentLoaded', function() {
  // Inicializa o mapa se estivermos na página do mapa
  initMap();
});