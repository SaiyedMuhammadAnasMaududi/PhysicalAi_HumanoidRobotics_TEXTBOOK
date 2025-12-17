#!/bin/bash

# Deployment Verification Script
# Tests backend API and frontend integration

set -e

BACKEND_URL=${1:-"http://localhost:8000"}
FRONTEND_URL=${2:-"http://localhost:3000"}

echo "🔍 Verifying Deployment"
echo "======================="
echo "Backend:  $BACKEND_URL"
echo "Frontend: $FRONTEND_URL"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
PASSED=0
FAILED=0

# Helper function for tests
test_endpoint() {
  local name=$1
  local url=$2
  local expected=$3

  echo -n "Testing $name... "

  response=$(curl -s -o /dev/null -w "%{http_code}" "$url" || echo "000")

  if [ "$response" = "$expected" ]; then
    echo -e "${GREEN}✓ PASS${NC} (HTTP $response)"
    ((PASSED++))
  else
    echo -e "${RED}✗ FAIL${NC} (Expected $expected, got $response)"
    ((FAILED++))
  fi
}

# Test backend endpoints
echo "🔧 Backend API Tests"
echo "--------------------"

test_endpoint "Root endpoint" "$BACKEND_URL/" "200"
test_endpoint "Health check" "$BACKEND_URL/health" "200"
test_endpoint "API health check" "$BACKEND_URL/api/health" "200"

# Test query endpoint with POST
echo -n "Testing query endpoint... "
query_response=$(curl -s -X POST "$BACKEND_URL/api/chatbot/query" \
  -H "Content-Type: application/json" \
  -d '{"query":"test","session_id":"test"}' \
  -w "%{http_code}" \
  -o /tmp/query_response.json || echo "000")

if [ "$query_response" = "200" ] || [ "$query_response" = "422" ]; then
  # 422 is acceptable (validation error), means endpoint is working
  echo -e "${GREEN}✓ PASS${NC} (HTTP $query_response)"
  ((PASSED++))
else
  echo -e "${RED}✗ FAIL${NC} (HTTP $query_response)"
  ((FAILED++))
fi

echo ""

# Test CORS headers
echo "🌐 CORS Configuration Test"
echo "--------------------------"

echo -n "Testing CORS headers... "
cors_headers=$(curl -s -I -X OPTIONS "$BACKEND_URL/api/chatbot/query" \
  -H "Origin: $FRONTEND_URL" \
  -H "Access-Control-Request-Method: POST" | grep -i "access-control" || echo "")

if [ -n "$cors_headers" ]; then
  echo -e "${GREEN}✓ PASS${NC}"
  echo "$cors_headers" | sed 's/^/  /'
  ((PASSED++))
else
  echo -e "${YELLOW}⚠ WARNING${NC} (No CORS headers found)"
  echo "  Make sure BACKEND_CORS_ORIGINS includes: $FRONTEND_URL"
fi

echo ""

# Test database connectivity (via health check)
echo "💾 Database Connectivity Test"
echo "-----------------------------"

if [ -f "/tmp/health_response.json" ]; then
  rm /tmp/health_response.json
fi

curl -s "$BACKEND_URL/health" > /tmp/health_response.json 2>&1 || true

if [ -f "/tmp/health_response.json" ]; then
  qdrant_status=$(cat /tmp/health_response.json | grep -o '"qdrant_status":"[^"]*"' | cut -d'"' -f4 || echo "unknown")
  neon_status=$(cat /tmp/health_response.json | grep -o '"neon_status":"[^"]*"' | cut -d'"' -f4 || echo "unknown")
  models_status=$(cat /tmp/health_response.json | grep -o '"models_status":"[^"]*"' | cut -d'"' -f4 || echo "unknown")

  echo -n "Qdrant status: "
  if [ "$qdrant_status" = "connected" ]; then
    echo -e "${GREEN}✓ Connected${NC}"
    ((PASSED++))
  else
    echo -e "${RED}✗ Not connected${NC} ($qdrant_status)"
    ((FAILED++))
  fi

  echo -n "Neon Postgres status: "
  if [ "$neon_status" = "connected" ]; then
    echo -e "${GREEN}✓ Connected${NC}"
    ((PASSED++))
  else
    echo -e "${RED}✗ Not connected${NC} ($neon_status)"
    ((FAILED++))
  fi

  echo -n "LLM Models status: "
  if [ "$models_status" = "available" ]; then
    echo -e "${GREEN}✓ Available${NC}"
    ((PASSED++))
  else
    echo -e "${RED}✗ Not available${NC} ($models_status)"
    ((FAILED++))
  fi
else
  echo -e "${RED}✗ Could not fetch health status${NC}"
  ((FAILED++))
fi

echo ""

# Frontend tests (if URL provided and not localhost)
if [[ "$FRONTEND_URL" != "http://localhost"* ]]; then
  echo "🖥️  Frontend Tests"
  echo "------------------"

  test_endpoint "Frontend homepage" "$FRONTEND_URL" "200"

  echo -n "Testing chatbot integration... "
  chatbot_check=$(curl -s "$FRONTEND_URL" | grep -i "chatbot\|assistant\|rag" || echo "")

  if [ -n "$chatbot_check" ]; then
    echo -e "${GREEN}✓ PASS${NC} (Chatbot components found)"
    ((PASSED++))
  else
    echo -e "${YELLOW}⚠ WARNING${NC} (Chatbot components not detected in HTML)"
  fi

  echo ""
fi

# Performance test
echo "⚡ Performance Test"
echo "-------------------"

echo -n "Testing response time... "
start_time=$(date +%s%N)
curl -s "$BACKEND_URL/health" > /dev/null
end_time=$(date +%s%N)
duration=$(( (end_time - start_time) / 1000000 ))  # Convert to milliseconds

if [ $duration -lt 1000 ]; then
  echo -e "${GREEN}✓ PASS${NC} (${duration}ms)"
  ((PASSED++))
elif [ $duration -lt 3000 ]; then
  echo -e "${YELLOW}⚠ SLOW${NC} (${duration}ms)"
else
  echo -e "${RED}✗ TOO SLOW${NC} (${duration}ms)"
  ((FAILED++))
fi

echo ""

# Summary
echo "📊 Test Summary"
echo "==============="
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
  echo -e "${GREEN}✅ All tests passed!${NC}"
  echo ""
  echo "🎉 Deployment verification successful!"
  echo ""
  echo "Next steps:"
  echo "  1. Test actual queries through the frontend"
  echo "  2. Monitor logs for any errors"
  echo "  3. Check Qdrant and Neon dashboards"
  echo "  4. Verify content is properly indexed"
  exit 0
else
  echo -e "${RED}❌ Some tests failed${NC}"
  echo ""
  echo "Troubleshooting:"
  echo "  1. Check backend logs for errors"
  echo "  2. Verify environment variables are set correctly"
  echo "  3. Ensure Qdrant and Neon databases are accessible"
  echo "  4. Check CORS configuration"
  exit 1
fi
