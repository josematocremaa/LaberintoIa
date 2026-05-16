#!/bin/bash

###############################################################################
# INSTALADOR AUTOMÁTICO - Controlador Khepera3 en Webots
# 
# Uso: bash INSTALL.sh [ruta_webots_project]
# 
# Ejemplo:
#   bash INSTALL.sh /home/usuario/mi_proyecto_webots
#   bash INSTALL.sh "C:\Users\Usuario\webots_project"  (Windows con GitBash)
###############################################################################

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Directorio de este script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  🚀 INSTALADOR CONTROLADOR KHEPERA3${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Validar que el usuario proporcionó ruta
if [ $# -eq 0 ]; then
    echo -e "${YELLOW}⚠️  Uso: bash INSTALL.sh [ruta_al_proyecto_webots]${NC}"
    echo ""
    echo "Ejemplos:"
    echo "  bash INSTALL.sh ~/mi_proyecto_webots"
    echo "  bash INSTALL.sh /home/usuario/webots_project"
    echo ""
    echo "📍 Si no tienes proyecto, primero crea una carpeta:"
    echo "  mkdir -p ~/webots_project/{controllers,worlds}"
    echo "  bash INSTALL.sh ~/webots_project"
    echo ""
    exit 1
fi

WEBOTS_PROJECT="$1"

# Validar que la ruta existe
if [ ! -d "$WEBOTS_PROJECT" ]; then
    echo -e "${RED}❌ ERROR: La carpeta '$WEBOTS_PROJECT' no existe${NC}"
    echo ""
    echo "Crea la carpeta primero:"
    echo "  mkdir -p '$WEBOTS_PROJECT'"
    exit 1
fi

echo -e "${BLUE}📂 Ruta del proyecto Webots:${NC} $WEBOTS_PROJECT"
echo ""

# Crear estructura de carpetas
echo -e "${BLUE}📁 Creando estructura de carpetas...${NC}"
mkdir -p "$WEBOTS_PROJECT/controllers/Grupo_N"
mkdir -p "$WEBOTS_PROJECT/worlds"
echo -e "${GREEN}✅ Carpetas creadas${NC}"
echo ""

# Copiar archivos
echo -e "${BLUE}📋 Copiando archivos...${NC}"

# Controlador básico
if [ -f "$SCRIPT_DIR/Grupo_N.py" ]; then
    cp "$SCRIPT_DIR/Grupo_N.py" "$WEBOTS_PROJECT/controllers/Grupo_N/Grupo_N.py"
    echo -e "${GREEN}✅ Grupo_N.py${NC}"
else
    echo -e "${RED}❌ No encontrado: Grupo_N.py${NC}"
fi

# Controlador avanzado
if [ -f "$SCRIPT_DIR/Grupo_N_advanced.py" ]; then
    cp "$SCRIPT_DIR/Grupo_N_advanced.py" "$WEBOTS_PROJECT/controllers/Grupo_N/Grupo_N_advanced.py"
    echo -e "${GREEN}✅ Grupo_N_advanced.py${NC}"
else
    echo -e "${YELLOW}⚠️  No encontrado: Grupo_N_advanced.py${NC}"
fi

# Controlador configurable
if [ -f "$SCRIPT_DIR/Grupo_N_configurable.py" ]; then
    cp "$SCRIPT_DIR/Grupo_N_configurable.py" "$WEBOTS_PROJECT/controllers/Grupo_N/Grupo_N_configurable.py"
    echo -e "${GREEN}✅ Grupo_N_configurable.py${NC}"
else
    echo -e "${YELLOW}⚠️  No encontrado: Grupo_N_configurable.py${NC}"
fi

# Configuración
if [ -f "$SCRIPT_DIR/config.py" ]; then
    cp "$SCRIPT_DIR/config.py" "$WEBOTS_PROJECT/controllers/Grupo_N/config.py"
    echo -e "${GREEN}✅ config.py${NC}"
else
    echo -e "${YELLOW}⚠️  No encontrado: config.py${NC}"
fi

# Mundo
if [ -f "$SCRIPT_DIR/maze_world.wbt" ]; then
    cp "$SCRIPT_DIR/maze_world.wbt" "$WEBOTS_PROJECT/worlds/maze_world.wbt"
    echo -e "${GREEN}✅ maze_world.wbt${NC}"
else
    echo -e "${RED}❌ No encontrado: maze_world.wbt${NC}"
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✅ INSTALACIÓN COMPLETADA${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Mostrar estructura
echo -e "${BLUE}📂 Estructura instalada:${NC}"
echo "$WEBOTS_PROJECT/"
echo "├── controllers/"
echo "│   └── Grupo_N/"
echo "│       ├── Grupo_N.py (controlador básico)"
echo "│       ├── Grupo_N_advanced.py (opcional)"
echo "│       ├── Grupo_N_configurable.py (opcional)"
echo "│       └── config.py (opcional)"
echo "└── worlds/"
echo "    └── maze_world.wbt"
echo ""

echo -e "${YELLOW}📖 PRÓXIMOS PASOS:${NC}"
echo ""
echo "1️⃣  Abre Webots"
echo "2️⃣  File → Open World"
echo "3️⃣  Navega a: $WEBOTS_PROJECT/worlds/"
echo "4️⃣  Abre: maze_world.wbt"
echo "5️⃣  Click derecho en 'Robot' → Properties"
echo "6️⃣  En 'controller' escribe: Grupo_N"
echo "7️⃣  Click ▶️ para ejecutar"
echo ""

echo -e "${BLUE}📚 Documentación:${NC}"
echo "  - QUICK_START.md  → Guía rápida"
echo "  - README.md       → Documentación técnica"
echo "  - SUMMARY.md      → Resumen completo"
echo ""

echo -e "${GREEN}¡Listo! El controlador está instalado${NC} 🚀"
