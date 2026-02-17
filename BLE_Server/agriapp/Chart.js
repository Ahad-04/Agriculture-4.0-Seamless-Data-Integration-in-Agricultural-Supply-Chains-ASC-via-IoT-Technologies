options: {
  responsive: true,
  plugins: {
    legend: { position: 'top' },
    tooltip: {
      callbacks: {
        label: function(ctx) {
          return ctx.dataset.label + ": " + ctx.parsed.y.toFixed(2);
        }
      }
    }
  },
  scales: {
    x: { 
      type: 'time',
      time: { unit: 'minute', tooltipFormat: 'dd MMM yyyy HH:mm:ss' }
    },
    y: {
      beginAtZero: true,
      suggestedMin: 0,
      suggestedMax: 100
    }
  }
}
